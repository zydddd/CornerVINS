#!/bin/bash
dataset_root="/home/zyd/File/CID-SIMS/"
results_root="/home/zyd/File/results/test/"

method_root="/CID-SIMS/"
config_name="cid.yaml"

file_name="test.txt"
time_name="time.txt"

dataset_list=(
"office_building/office/office_1/"
"office_building/office/office_2/"
"office_building/office/office_3/"
"office_building/floor14/floor14_1/"
"office_building/floor14/floor14_3/"
"office_building/downstairs/14-13-12/"
"office_building/floor3/floor3_1/"
"office_building/floor3/floor3_2/"
"office_building/floor3/floor3_3/"
"office_building/floor13/floor13_1/"
"office_building/floor13/floor13_2/"
"apartment/apartment1/apartment1_2/"
"apartment/apartment1/apartment1_3/"
"apartment/apartment2/apartment2_2/"
"apartment/apartment3/apartment3_3/"
)

for folder in "${dataset_list[@]}"
do

  save_path=$results_root$method_root$folder
  if [ ! -d $save_path  ];then
    mkdir -p $save_path
  fi
#  mkdir -p $save_path"segs/"
#  mkdir -p $save_path"depth/"
#  mkdir -p $save_path"tracked/"
#  mkdir -p $save_path"historical/"
#  mkdir -p $save_path"corners/"

  ./bin/test_cid ./config/ $config_name $dataset_root$folder $save_path $file_name $time_name
  echo "$save_path finished."
  echo ""
  echo ""
done