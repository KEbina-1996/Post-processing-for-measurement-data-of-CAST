# Post-processing-for-measurement-data-of-CAST
This program is a post-processing for measurement data of practical laparoscopic surgical training, such as cadaver surgical training described in the paper [1]. To measure the motion of surgical instruments in this training environment with many obstacles and narrow measurement space, the trinocular motion capture system was adopted in this study. However, the measurement data contain some noise and outliers because this system only measures the instrument motion from one direction. Therefore, three outlier removal methods and interpolation and smoothing methods were developed in this study. This is an example program for post-processing.

## Files
- main.py: Sample code for applying post-processing to measurement data.
- dataProcessFunc.py: The functions for data processing.
- targetFile.csv: Target file name, cannula position, and cadaver position were recorded. This file is used as the input file for the program.
- mocap/: Input file directory. Sample measurement data is available for testing.
- calc/: Output file directory

## Usage
1. Copy these files to your computer.
2. Run "main.py".
3. Then the processed file will be stored in calc/.

## File format
- input data
- 
