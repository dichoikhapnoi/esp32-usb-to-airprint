#!/bin/bash
# Everything that can be verified without hardware.
#   tools/run_tests.sh               host unit tests + test page decode
#   GUTENPRINT=1 tools/run_tests.sh  + byte-for-byte comparison with Gutenprint
#   QEMU_XTENSA=... tools/run_tests.sh  + firmware dry run on emulated ESP32-S3
#   AIRPRINT=1 tools/run_tests.sh    + AirPrint server on this Mac: IPP conformance
#                                      (ipptool) and printing URF / PWG documents
#                                      rendered by macOS's own filters, and the
#                                      web UI's live state
#   WEB=1 tools/run_tests.sh         + the web UI in headless Chrome against the
#                                      host server (print, Process)
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=host/out
mkdir -p $OUT
OUT=$(cd $OUT && pwd)

echo "== host unit tests"
make -C host -s
host/bjl_host selftest
host/capt_host selftest | tail -2
host/oled_host selftest

echo "== status screen (all screens in $OUT/oled.png)"
rm -f $OUT/oled-*.pbm
host/oled_host render $OUT
python3 tools/oled_preview.py $OUT $OUT/oled.png

echo "== web UI unit tests (web/tests)"
if [ -d web/node_modules ]; then
    (cd web && node --test tests/*.test.ts 2>&1 | grep -E "^ℹ (pass|fail)|^✖" | tr '\n' ' '; echo)
    # the browser's URF, decoded by the independent Python decoder
    python3 - <<'PY'
import sys, numpy as np
sys.path.insert(0, 'tools')
from raster_to_png import pages
src = np.fromfile('host/out/web-color.rgba', dtype=np.uint8).reshape(500, 333, 4)[:, :, :3]
(img, w, h, bpp, dpi), = list(pages(open('host/out/web-color.urf', 'rb').read()))
print('URF from the browser encoder, decoded by raster_to_png.py:',
      'identical' if (img.reshape(h, w, 3) == src).all() else 'DIFFERENT')
PY
else
    echo "skipped: no web/node_modules (cd web && npm ci)"
fi

echo "== test page streams (decoded previews in $OUT/)"
for dpi in 300 600; do
    host/bjl_host testpage $dpi $OUT/testpage$dpi.prn
    python3 tools/bjl_decode.py $OUT/testpage$dpi.prn --png $OUT/testpage$dpi.png --quiet | tail -6
done

if [ -n "${GUTENPRINT:-}" ]; then
    echo "== comparison with Gutenprint's MG2500 driver"
    [ -f tools/gutenprint_ref/build/ref600.prn ] || tools/gutenprint_ref/build_ref.sh
    R=tools/gutenprint_ref/build
    [ -f $R/ref300_2pages.prn ] && [ -f $R/iP2700/ref300_2pages.prn ] || tools/gutenprint_ref/build_ref.sh
    python3 tools/compare_gutenprint.py $R/ref300.prn 300
    python3 tools/compare_gutenprint.py $R/ref600.prn 600
    python3 tools/compare_gutenprint.py $R/ref300_letter.prn 300 --paper letter
    python3 tools/compare_gutenprint.py $R/ref300_2pages.prn 300
    echo "== comparison with Gutenprint, the other stream variants (canon_bjl.h models)"
    for m in MG2900 iP2800 E3100 MP230 iP2700; do
        echo "-- $m"
        python3 tools/compare_gutenprint.py $R/$m/ref300.prn 300 --model $m
        python3 tools/compare_gutenprint.py $R/$m/ref600.prn 600 --model $m
        python3 tools/compare_gutenprint.py $R/$m/ref300_letter.prn 300 --paper letter --model $m
        python3 tools/compare_gutenprint.py $R/$m/ref300_2pages.prn 300 --model $m
    done
fi

if [ -n "${AIRPRINT:-}" ]; then
    echo "== AirPrint server (host build)"
    PORT=8631
    URI=ipp://localhost:$PORT/ipp/print
    pkill -f "airprint_host 863[12]" 2>/dev/null || true     # stale servers from an aborted run
    PPD=tools/testdata/raster_test.ppd
    DOCS=/usr/share/cups/ipptool
    (cd host && exec ./airprint_host $PORT > out/airprint.log 2>&1) &
    SERVER=$!
    trap 'kill $SERVER 2>/dev/null' EXIT
    sleep 1
    for t in ipp-1.1.test ipp-2.0.test ipp-everywhere.test; do
        r=$(ipptool -I -t $URI $t 2>&1) || true    # exits 1 on any failed test
        echo "$t: $(grep -c '\[PASS\]' <<<"$r") pass, $(grep -c '\[FAIL\]' <<<"$r") fail"
        grep -A3 '\[FAIL\]' <<<"$r" | grep -E "FAIL|EXPECTED" || true
    done
    echo "(expected: ipp-everywhere fails only on image/jpeg, which AirPrint never uses)"
    rm -f host/out/job-*.prn
    cupsfilter -p $PPD -m image/urf $DOCS/document-a4.pdf > $OUT/doc-a4.urf 2>/dev/null
    cupsfilter -p $PPD -m image/pwg-raster $DOCS/document-a4.pdf > $OUT/doc-a4.pwg 2>/dev/null
    cupsfilter -p $PPD -m image/urf -o PageSize=Letter -o Resolution=300dpi $DOCS/onepage-letter.pdf > $OUT/letter300.urf 2>/dev/null
    cupsfilter -p $PPD -m image/urf -o ColorModel=Gray -o Resolution=300dpi $DOCS/onepage-a4.pdf > $OUT/gray300.urf 2>/dev/null
    for f in doc-a4.urf doc-a4.pwg letter300.urf gray300.urf; do
        type=image/urf; [[ $f == *.pwg ]] && type=image/pwg-raster
        echo "print $f: $(ipptool -t -f $OUT/$f -d filetype=$type $URI print-job.test 2>&1 | grep -oE 'PASS|FAIL')"
    done
    JOBS=($(ls host/out/job-*.prn | sort -t- -k2 -n))  # in print order: urf, pwg, letter, gray
    cmp -s ${JOBS[0]} ${JOBS[1]} && echo "URF and PWG of the same document: identical printer data" \
        || echo "URF and PWG outputs DIFFER"
    for j in host/out/job-*.prn; do
        python3 tools/bjl_decode.py $j --png ${j%.prn}.png --quiet | grep -E "^page|DECODE ERROR"
    done
    kill $SERVER; wait $SERVER 2>/dev/null || true
    echo "== cancel while printing (slow printer emulation)"
    PORT=8632
    URI=ipp://localhost:$PORT/ipp/print
    (cd host && AIRPRINT_HOST_SLOW_US=20000 exec ./airprint_host $PORT > out/airprint-cancel.log 2>&1) &
    SERVER=$!
    sleep 1
    ipptool -t -f $OUT/doc-a4.urf -d filetype=image/urf $URI print-job.test > $OUT/printjob.log 2>&1 &
    CLIENT=$!
    sleep 3
    curl -s -o $OUT/oled-live-1-printing.pbm http://localhost:$PORT/dev/oled.pbm
    ipptool -t $URI tools/testdata/cancel-active-job.test 2>&1 | grep -E "PASS|FAIL|EXPECTED" || true
    wait $CLIENT || true
    curl -s -o $OUT/oled-live-2-canceled.pbm http://localhost:$PORT/dev/oled.pbm
    echo "original Print-Job client: $(grep -oE 'PASS|FAIL' $OUT/printjob.log)"
    kill $SERVER; wait $SERVER 2>/dev/null || true

    echo "== web UI: live state, events"
    PORT=8633
    (cd host && exec ./airprint_host $PORT > out/airprint-state.log 2>&1) &
    SERVER=$!
    sleep 1
    W=http://localhost:$PORT
    js=$(curl -s --compressed $W/ | grep -oE '/assets/index-[^"]+\.js' | head -1)
    etag=$(curl -s -D - -o /dev/null $W/ | tr -d '\r' | awk '/^ETag:/ {print $2}')
    echo "page $(curl -s -o /dev/null -w '%{http_code}' $W/), app $(curl -s --compressed -o /dev/null -w '%{http_code}' $W$js)," \
         "again with its ETag $(curl -s -o /dev/null -w '%{http_code}' -H "If-None-Match: $etag" $W/)"
    curl -s -N --max-time 3 $W/events > $OUT/events.txt || true
    python3 - $OUT/events.txt <<'PY'
import json, sys
ev = [json.loads(l[6:]) for l in open(sys.argv[1]) if l.startswith('data: ')]
print('events: %d snapshot(s), printer %s' % (len(ev), ev[-1]['printer']['state']))
PY
    kill $SERVER; wait $SERVER 2>/dev/null || true
    # the screen as the server's state drew it: printing, canceled
    python3 - $OUT <<'PY'
import sys
from PIL import Image
names = ['1-printing', '2-canceled']
imgs = [Image.open(f'{sys.argv[1]}/oled-live-{n}.pbm').convert('1') for n in names]
lit = [sum(1 for v in i.crop((0, 23, 128, 37)).getdata() if v == 0) for i in imgs]  # headline row
distinct = len({i.tobytes() for i in imgs})
print('status screen from the live state: %d of 2 screens with a headline, %d distinct' % (sum(n > 50 for n in lit), distinct))
PY
    python3 tools/oled_preview.py $OUT $OUT/oled.png > /dev/null
    trap - EXIT
fi

if [ -n "${WEB:-}" ]; then
    echo "== web UI in headless Chrome (screenshots in $OUT/ui-*.png)"
    PORT=8641
    mkdir -p $OUT/e2e
    gunzip -c /usr/share/cups/ipptool/document-a4.pdf > $OUT/e2e/document-a4.pdf
    gunzip -c /usr/share/cups/ipptool/color.jpg > $OUT/e2e/color.jpg
    pkill -f "airprint_host $PORT" 2>/dev/null || true
    rm -f host/out/job-*.prn
    (cd host && exec ./airprint_host $PORT > out/airprint-web.log 2>&1) &
    SERVER=$!
    trap 'kill $SERVER 2>/dev/null' EXIT
    sleep 1
    (cd web && BASE=http://localhost:$PORT node tests/e2e.mjs 2>&1 | grep -E "^- |^e2e|Error|rror:" | head -20)
    for j in host/out/job-*.prn; do
        python3 tools/bjl_decode.py $j --png ${j%.prn}.png --scale 4 --quiet | grep -E "^page|DECODE ERROR" | sed "s|^|  $(basename $j): |"
    done
    kill $SERVER; wait $SERVER 2>/dev/null || true
    trap - EXIT
fi

if [ -n "${QEMU_XTENSA:-}" ]; then
    echo "== firmware dry run in QEMU"
    tools/qemu_selftest.sh
fi

echo "== firmware build"
pio run -e esp32s3 | grep -E "RAM:|Flash:|SUCCESS|FAILED"
