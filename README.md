Here is a professional README.md file for the given program:

# Angle Locator Position Estimator 

This program estimates the position of a tag device using angle measurements from multiple locator devices.

## Input
The input is read from an input text file. Each line of the file contains a position estimation request with multiple measurements separated by semicolons.

Each measurement contains four comma-separated values:

- x: The x coordinate of the locator 
- y: The y coordinate of the locator
- rotz: The rotation of the locator around the z-axis
- azimuth: The angle from the locator's x-axis to the ray cast from the locator to the tag

All angles are given in degrees.

## Output 
The output is written to an output text file. Each line of the output contains the estimated position and error for one request. The format of each line is:

`x, y, err`

If a position cannot be estimated for a request due to insufficient measurements, "fail" is written to the output, optionally followed by a reason.

## Method
The position is estimated using the RANSAC algorithm. Random pairs of measurements are used to calculate candidate positions. The position with the most inliers within a maximum error threshold is selected as the final estimated position.

## Algorithm
The program uses the RANSAC (RANdom SAmple Consensus) algorithm to estimate the tag position from the measurements. The steps of the algorithm are:

1. Randomly select two measurements from the input 
2. Calculate the candidate position using the two selected measurements 
3. Count the number of inliers for the candidate position within a maximum error threshold 
4. Keep track of the candidate position with the most inliers so far
5. Repeat steps 1-4 a fixed number of times (10000 iterations are used in this program)
6. The candidate position with the most inliers is selected as the final estimated position

To calculate the candidate position from two measurements, the following steps are used:

1. Convert the rotz and azimuth angles to radians 
2. Calculate the slope of the angle measurement for each of the two selected measurements 
3. Calculate the y-intercept for each of the two selected measurements 
4. The two lines represented by the slopes and intercepts will intersect at a point. This intersection point is the candidate position. 
5. The intersection point is calculated by equating the two lines and solving for x and y.

To count the number of inliers for a candidate position, the following is done:

1. For each measurement in the input, calculate the expected y position on the locator's line of sight 
2. Calculate the error between the expected y position and the y coordinate of the candidate position 
3. If the error is within the maximum error threshold (8.3 degrees is used in this program), increment the inlier count
4. Keep a sum of all the inlier errors to calculate the average error later 

The final estimated position and average error are written to the output file. If a position cannot be estimated due to insufficient measurements (less than 2), "fail" is written to the output.

## Requirements
- C++ 11 or higher

## Usage
1. Update the input and output file paths in `src/main.cpp`
2. Compile the program using `g++ src/main.cpp -o locator` 
3. Run the program using `./locator`

The input file will be read, positions estimated, and the output written to the output file.

## License
[MIT](https://choosealicense.com/licenses/mit/)
