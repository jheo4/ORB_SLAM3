#!/bin/bash
pathDatasetEuroc='/home/mnt/Data/EuRoC'

#------------------------------------
# Monocular Examples
#echo "Launching V101 with Monocular sensor"
#./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Monocular/EuRoC_TimeStamps/V101.txt dataset-V101_mono

#------------------------------------
# Stereo Examples
#echo "Launching V101 with Stereo sensor"
#./Stereo/stereo_euroc ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Stereo/EuRoC_TimeStamps/V101.txt dataset-V101_stereo

#------------------------------------
echo "Launching V101 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_test ../Vocabulary/ORBvoc.txt ./Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Monocular-Inertial/EuRoC_TimeStamps/V101.txt dataset-V101_monoi

#------------------------------------
#echo "Launching V101 with Stereo-Inertial sensor"
#./Stereo-Inertial/stereo_inertial_euroc ../Vocabulary/ORBvoc.txt ./Stereo-Inertial/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Stereo-Inertial/EuRoC_TimeStamps/V101.txt dataset-V101_stereoi

