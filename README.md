# Starfield


Starfield is a library for calibrating cameras attached to gantry systems, using the motion of the machine to provide geometric references.


## Library

TODO: how do I install and use the library?

## Calibration Script/Workflow

A pre-written script for camera calibration is installed as *starfield-calibrate*, with source provided in *calibrate.py*. Use caution when first using this script, especially with custom machine drivers and expensive cameras - although it has been written to be predictable and safe, crashes are always a possibility. For bespoke applications, this script should be considered a starting point for a custom calibration workflow.

```
Usage: calibrate.py [OPTIONS] OUTPUT

Options:
  --cv-camera INTEGER        OpenCV VideoCapture ID for the desired camera -
                             see OpenCV documentation for how to find this.
  --resolution TEXT          Desired camera capture resolution, in the format
                             WIDTHxHEIGHT
  --focus-height FLOAT       Initial guess for the focus plane. If not
                             provided, a live camera view and jog interface
                             will appear before the autofocus pass.
  --focus-range FLOAT        How larger of a range should the autofocus pass
                             search on either side of the focus height? If
                             both this argument and --focus-height are
                             provided, the autofocus pass will run without
                             user input.
  --home / --no-home         Should the machine home/reference its z axis
                             before starting the calibration process?
  --target-thickness FLOAT   How far is the target surface about the work
                             surface of the machine? This parameter doesn't
                             impact the calibration process, but is applied to
                             saved focal plane offsets.
  --tolerance FLOAT          How precise should we aim to be during the
                             autofocus pass?
  --discrete / --continuous  Should we use a discrete, step-wise autofocus
                             pass (slow, insensitive to controller interface)
                             or a continuous (faster, more precise, requires
                             better position feedback) one?
  --field-size FLOAT         Initial guess for the length of the longest side
                             of the camera's field of view.
  --help                     Show this message and exit.
```

## Calibration Targets

Although the calibration process doesn't require any specific reference geometry, it does perform best with a flat, high-contrast target with lots of corners to track. One of the cheapest, most-precise ways to produce an object satisfying these requirements is with printed circuit boards - particularly a PCB process with high-resolution silkscreen  (or bare ENIG metal) on a matte black solder mask or substrate. *target/startfield-target.py* generates KiCAD footprints with a multi-scale Poisson-disc layout of many squares, ranging in size down to a stated fabrication process tolerance. Targets may also be purchased from [Oshpark](https://oshpark.com/shared_projects/1K8nM6ZZ) (be sure to select the "After Dark" option, for maximum contrast), or printed on a high-resolution printer.

Depending on the camera magnification, small text, black pen scribbles on paper, or any speckled surface also works quite well. 

