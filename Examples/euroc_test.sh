#!/bin/bash
pathDatasetEuroc='/Datasets/EuRoC' #Example, it is necesary to change it by the dataset path

./Monocular-Inertial/mono_inertial_test ../Vocabulary/ORBvoc.txt ./Monocular-Inertial/EuRoC.yaml /home/mnt/Data/EuRoC/V101 ./Monocular-Inertial/EuRoC_TimeStamps/V101.txt dataset-V101_monoi

