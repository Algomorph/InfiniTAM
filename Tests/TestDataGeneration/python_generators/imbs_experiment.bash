#!/bin/bash

#############################################################################
# Bash script to perform a parameter search for background subtraction      #
#############################################################################

BASE_OUTPUT_DIR=/mnt/Data/Reconstruction/real_data/snoopy/imbs_mask_experiment

FG_THRESHOLD=14
ASSOCIATION_THRESHOLD=5
NUM_SAMPLES=30
ALPHA=0.40
BETA=0.95
TAU_S=45
TAU_H=60
# MIN_AREA=1.15
# PERSISTENCE_PERIOD=10000
MF=OFF
PMF=ON
PMKS=5

ALPHA_STR=$(echo "$ALPHA * 100 / 1" | bc)
BETA_STR=$(echo "$BETA * 100 / 1" | bc)
# MIN_AREA_STR=$(echo "$MIN_AREA * 100 / 1" | bc)

MF_ARG=
if [ "$MF" =  "ON" ]
then
  MF_ARG="-m"
fi

PMF_ARG=
if [ "$PMF" =  "ON" ]
then
  PMF_ARG="-pm"
fi

FRAMES_DIR=/mnt/Data/Reconstruction/real_data/snoopy/frames
DEPTH_FRAMES_DIR=/mnt/Data/Reconstruction/real_data/snoopy/frames
BACKGROUND_DIR=/mnt/Data/Reconstruction/real_data/snoopy/color_bg



makedir -p "$BASE_OUTPUT_DIR"

# single run
#OUT_PATH="$BASE_OUTPUT_DIR"/fgt_"$FG_THRESHOLD"_at_"$ASSOCIATION_THRESHOLD"_ns_"$NUM_SAMPLES"_a_"$ALPHA_STR"_b_"$BETA_STR"_ts_"$TAU_S"_th_"$TAU_H"_mf_"$MF"_pmf_"$PMF"_pmks_"$PMKS".avi
#COMMAND="python3.6 ./make_imbs_masks.py -f=$FRAMES_DIR -bf=$BACKGROUND_DIR -df=$DEPTH_FRAMES_DIR  -o=$OUT_PATH -lf -v \\
#--clip_depth --near_clip=500 --far_clip=1000 --fps=30 --fg_threshold=$FG_THRESHOLD \\
#--association_threshold=$ASSOCIATION_THRESHOLD --num_samples=$NUM_SAMPLES --alpha=$ALPHA --beta=$BETA --tau_s=$TAU_S \\
#--tau_h=$TAU_H $MF_ARG $PMF_ARG -pmks=$PMKS"
#echo "$COMMAND"
#eval "$COMMAND"


# batch run
#SUB_OUTPUT_DIR="$BASE_OUTPUT_DIR/PMKS\ variations\ open-close"
#makedir -p "$SUB_OUTPUT_DIR"
#
#for PMKS in $(seq 3 2 7)
#do
#  ALPHA_STR=$(echo "$ALPHA * 100 / 1" | bc)
#  BETA_STR=$(echo "$BETA * 100 / 1" | bc)
#  OUT_PATH="$SUB_OUTPUT_DIR"/fgt_"$FG_THRESHOLD"_at_"$ASSOCIATION_THRESHOLD"_ns_"$NUM_SAMPLES"_a_"$ALPHA_STR"_b_"$BETA_STR"_ts_"$TAU_S"_th_"$TAU_H"_mf_"$MF"_pmf_"$PMF"_pmks_"$PMKS".avi
#  COMMAND="python3.6 ./make_imbs_masks.py -f=$FRAMES_DIR -bf=$BACKGROUND_DIR -df=$DEPTH_FRAMES_DIR  -o=$OUT_PATH -lf -v \\
#--clip_depth --near_clip=500 --far_clip=1000 --fps=30 --fg_threshold=$FG_THRESHOLD \\
#--association_threshold=$ASSOCIATION_THRESHOLD --num_samples=$NUM_SAMPLES --alpha=$ALPHA --beta=$BETA --tau_s=$TAU_S \\
#--tau_h=$TAU_H $MF_ARG $PMF_ARG -pmks=$PMKS"
#  echo "$COMMAND"
#  eval "$COMMAND"
#done

# final run
OUT_PATH="$BASE_OUTPUT_DIR"/fgt_"$FG_THRESHOLD"_at_"$ASSOCIATION_THRESHOLD"_ns_"$NUM_SAMPLES"_a_"$ALPHA_STR"_b_"$BETA_STR"_ts_"$TAU_S"_th_"$TAU_H"_mf_"$MF"_pmf_"$PMF"_pmks_"$PMKS"
COMMAND="python3.6 ./make_imbs_masks.py -f=$FRAMES_DIR -bf=$BACKGROUND_DIR -df=$DEPTH_FRAMES_DIR  -o=$OUT_PATH \\
--clip_depth --near_clip=500 --far_clip=1000 --fps=30 --fg_threshold=$FG_THRESHOLD \\
--association_threshold=$ASSOCIATION_THRESHOLD --num_samples=$NUM_SAMPLES --alpha=$ALPHA --beta=$BETA --tau_s=$TAU_S \\
--tau_h=$TAU_H $MF_ARG $PMF_ARG -pmks=$PMKS"
echo "$COMMAND"
eval "$COMMAND"




