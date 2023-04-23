# Compiling the C++ binary
This repo does not contain the original source code for the ORB-SLAM C++ binary (which is inside packages/slam_node/src/ and named orb_slam_server), but assumes the program is already compiled in another repository.

# Getting large files from Google Drive
Due to file size limits the libraries and ORBvoc.txt files can't be placed inside this repo. I have packed the entire repo to a tar file "Pj428.tar" on my google drive folder instead: https://drive.google.com/drive/folders/1t0Ybwq3VYVytOJXkouMso00pHGD_Sf_8

# Running The Code
The set up here is similar to how we run the code in cmput 412. To build the code
```
dts devel build -f
```
To run the code remotely on laptop and connect to the robot 'csc229XX':
```
dts devel run -R csc229XX
```
