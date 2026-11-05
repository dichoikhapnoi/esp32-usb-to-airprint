// Two-sided printing: the duplex byte of the URF page header is what the printer server reads
// (components/airprint/raster.c: 1 one-sided, 2 two-sided long edge, 3 short edge).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { urfFileHeader, urfPageHeader } from '../src/lib/urf.ts';

test('URF page header carries the sides', () => {
  const mode = (sides?: 'one' | 'long' | 'short') =>
    urfPageHeader({ width: 8, height: 8, dpi: 300, gray: true, sides })[2];
  assert.equal(mode(), 1);
  assert.equal(mode('one'), 1);
  assert.equal(mode('long'), 2);
  assert.equal(mode('short'), 3);
});

test('the rest of the page header does not depend on the sides', () => {
  const a = urfPageHeader({ width: 2480, height: 3508, dpi: 300, gray: false });
  const b = urfPageHeader({ width: 2480, height: 3508, dpi: 300, gray: false, sides: 'long' });
  assert.equal(a.length, 32);
  assert.deepEqual([...a.subarray(3)], [...b.subarray(3)]);
  assert.deepEqual([...a.subarray(0, 2)], [...b.subarray(0, 2)]);
  assert.equal(urfFileHeader(4).length, 12);
});
