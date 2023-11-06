# Post-processing-for-measurement-data-of-CAST
This program is a post-processing for measurement data of practical laparoscopic surgical training, such as cadaver surgical training described in the paper [1].
To measure the motion of surgical instruments in this training environment with many obstacles and narrow measurement space, the trinocular motion capture system was adopted in this study.
However, the measurement data contain some noise and outliers because this system only measures the instrument motion from one direction.
Therefore, three outlier removal methods and interpolation and smoothing methods were developed in this study. This is an example program for post-processing.

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
- input data (csv file):
This file must contain the time stamp, tip position (X, Y, Z), and orientation in quaternion form (W, X, Y, Z) of the surgical instruments.
The file header consists of four lines. See the sample file for detailed formatting (mocap/).
The sample file contains motion data for six surgical instruments in the surgical training.
Note that due to the file size limitations of the github, up to 1000 seconds of data is stored.
- targetFile.csv:
This file specifies the data file to be processed and also contains additional information about the file.
  - filename: Designates the name of the file to be processed. Note that the target file must be saved under ″mocap″ directory.
  - processFg: This is a flag to process or not. Setting this value to 1 will process the target file; setting it to 0 will ignore it.
  - cadaverVec: Specifies the orientation of the cadaver body. Since the head orientation changes between right and left nephrectomy, this option is used to adjust the orientation.
  - trocar1_x,y,z to trocar3_x,y,z: The port (cannula) position on the cadaver body. The 3D positions of the three ports, including the camera port, must be recorded.
  - cadaver1_x,y,z to cadaver3_x,y,z: The 3D position of the three anatomical feature of the cadaver body (shoulder, anterior superior iliac spine, and knee). These position were recorded by attaching markers to the cadaver body.
