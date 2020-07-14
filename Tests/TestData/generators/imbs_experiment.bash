#!/bin/bash

BASE_OUTPUT_DIR=/mnt/Data/Reconstruction/real_data/snoopy/imbs_mask_experiment

FG_THRESHOLD=15
ASSOCIATION_THRESHOLD=5
NUM_SAMPLES=30
ALPHA=0.65
BETA=1.15
TAU_S=60
TAU_H=40
# MIN_AREA=1.15
# PERSISTENCE_PERIOD=10000
MF=ON

ALPHA_STR=$(echo "$ALPHA * 100 / 1" | bc)
BETA_STR=$(echo "$BETA * 100 / 1" | bc)
# MIN_AREA_STR=$(echo "$MIN_AREA * 100 / 1" | bc)

MF_ARG=
if [ "$MF" =  "ON" ]
then
  MF_ARG="-m"
fi

OUT_PATH="$BASE_OUTPUT_DIR"/IMBS/fgt_"$FG_THRESHOLD"_at_"$ASSOCIATION_THRESHOLD"_ns_"$NUM_SAMPLES"_a_"$ALPHA_STR"_b_"$BETA_STR"_ts_"$TAU_S"_th_"$TAU_H"_mf_"$MF".avi
echo "$OUT_PATH"

FRAMES_DIR=/mnt/Data/Reconstruction/real_data/snoopy/frames
BACKGROUND_DIR=/mnt/Data/Reconstruction/real_data/snoopy/color_bg

#makedir -p "$OUT_PATH"

COMMAND="python3.6 ./make_imbs_masks.py -o="$OUT_PATH" -f="$FRAMES_DIR" -bf="$BACKGROUND_DIR" -v --fps=30 --fg_threshold=${FG_THRESHOLD} \\
--association_threshold=${ASSOCIATION_THRESHOLD} --num_samples=${NUM_SAMPLES} --alpha=${ALPHA} --beta=${BETA} --tau_s=$TAU_S \\
--tau_h=$TAU_H ${MF_ARG}"

echo "$COMMAND"
eval "$COMMAND"


