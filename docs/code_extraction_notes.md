# Code Extraction Notes

The source code was extracted from the graduation project PDF appendices.

## Source locations in the PDF

- Appendix A: MATLAB code, pages 91-99 of the PDF.
- Appendix B: Arduino code, pages 100-123 of the PDF.

## Cleanups applied

The PDF text extraction introduced line-wrapping problems. The following cleanups were applied:

- Repaired split MATLAB words such as `ForegroundPolarity` and `LineWidth`.
- Repaired MATLAB syntax such as `for i = 1:num` and `[objNum, objLabel] = findObjType(...)`.
- Split helper functions into `findObjType.m` and `CallPlaceAng.m`.
- Repaired Arduino line wrapping in function signatures and comments.
- Replaced some single `&`/`|` logical checks with `&&`/`||` where appropriate.
- Preserved original variable names where possible, including misspelled names such as `reciveMatrix`, `PosAchive`, and `jearkCancel` to avoid breaking code references.

## Important TODOs

1. Replace `calibrationFile`, `cameraName`, and `serialPort` in MATLAB.
2. Replace placeholder bin joint angles in `CallPlaceAng.m`.
3. Verify Arduino pin mapping against the real wiring.
4. Compile the Arduino code before uploading.
5. Test with motor power disabled before full operation.
