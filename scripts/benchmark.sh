#!/bin/bash
# benchmark.sh [ci] [$app]
# Pass the option "ci" if there is no QuestaSim installed
# Pass the name of the app to benchmark
# If no app is passed, all the apps are benchmarked


# Python in use
PYTHON=python3

# Is this exectued by the CI?
if [ "$1" == "ci" ]
then
    ci=1
    # If there is a program
    shift
else
    ci=0
fi

# Include Ara's configuration
if [ -z ${config} ]; then
    if [ -z ${ARA_CONFIGURATION} ]; then
        config=default
    else
        config=${ARA_CONFIGURATION}
    fi
fi

tmpscript=`mktemp`
sed "s/ ?= /=/g" config/${config}.mk > $tmpscript
source ${tmpscript}

############
## MATMUL ##
############

matmul() {
  # Measure the runtime of the following kernels
  for kernel in imatmul fmatmul; do

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    # Measure the following matrix sizes
    for size in 4 8 16 32 64 128; do

      tempfile=`mktemp`

      # Clean, and then generate the correct matrix and filter
      make -C apps/ clean

      # Standard system
      config=${config} ENV_DEFINES="-DSIZE=$size -D${kernel^^}=1" \
             make -C apps/ bin/benchmarks
      make -C hardware/ simv app=benchmarks > $tempfile || exit
      # Extract the cycle count and calculate performance
      cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
      ./scripts/performance.py $kernel "$size" $cycles >> ${kernel}_${nr_lanes}.benchmark

      if [ "$ci" == 0 ]; then
        # System with ideal dispatcher
        config=${config} ENV_DEFINES="-DSIZE=$size -D${kernel^^}=1" \
               make -C apps/ bin/benchmarks.ideal
        touch -a hardware/build
        config=${config} make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
        # Extract the cycle count and calculate performance
        cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$size" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
      fi
    done
  done
}

################
## CONV2D 3x3 ##
################

conv2d() {
  # Measure the runtime of the following kernels
  for kernel in iconv2d fconv2d; do

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    # Measure the following matrix and filter sizes
    # The input image is also padded, and the max vl is 128
    # MAXVL_M2_64b - F_MAX + 1 = 128 - 7 + 1 = 122 is the max number of elements
    # Actually 120, since it must be divible by 4
    for msize in 4 8 16 32 64 112; do
      for fsize in 3; do
        tempfile=`mktemp`

        # Clean, and then generate the correct matrix and filter
        make -C apps/ clean

        mkdir -p apps/benchmarks/data
        ${PYTHON} apps/$kernel/script/gen_data.py $msize $fsize > apps/benchmarks/data/data.S

        # Standard System
        config=${config} ENV_DEFINES="-D${kernel^^}=1" \
               make -C apps/ bin/benchmarks
        make -C hardware/ simv app=benchmarks > $tempfile || exit
        # Extract the performance
        cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$msize $fsize" $cycles >> ${kernel}_${nr_lanes}.benchmark

        if [ "$ci" == 0 ]; then
          # System with ideal dispatcher
          config=${config} ENV_DEFINES="-D${kernel^^}=1" \
                 make -C apps/ bin/benchmarks.ideal
          touch -a hardware/build
          config=${config} make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
          # Extract the performance
          cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
          ./scripts/performance.py $kernel "$msize $fsize" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
        fi
      done
    done
  done
}

################
## CONV3D 7x7 ##
################

conv3d() {
  # Measure the runtime of the following kernels
  for kernel in fconv3d; do

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    # Measure the following matrix and filter sizes
    # The input image is also padded, and the max vl is 128
    # MAXVL_M2_64b - F_MAX + 1 = 128 - 7 + 1 = 122 is the max number of elements
    # Actually 120, since it must be divible by 4
    for msize in 4 8 16 32 64 112; do
      for fsize in 7; do
        tempfile=`mktemp`

        # Clean, and then generate the correct matrix and filter
        make -C apps/ clean

        mkdir -p apps/benchmarks/data
        ${PYTHON} apps/$kernel/script/gen_data.py $msize $fsize > apps/benchmarks/data/data.S

        # Standard System
        config=${config} ENV_DEFINES="-D${kernel^^}=1" \
               make -C apps/ bin/benchmarks
        make -C hardware/ simv app=benchmarks > $tempfile || exit
        # Extract the performance
        cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$msize $fsize" $cycles >> ${kernel}_${nr_lanes}.benchmark

        if [ "$ci" == 0 ]; then
          # System with ideal dispatcher
          config=${config} ENV_DEFINES="-D${kernel^^}=1" \
                 make -C apps/ bin/benchmarks.ideal
          touch -a hardware/build
          config=${config} make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
          # Extract the performance
          cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
          ./scripts/performance.py $kernel "$msize $fsize" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
        fi
      done
    done
  done
}

##############
## Jacobi2d ##
##############

jacobi2d() {
  # Measure the runtime of the following kernels
  for kernel in jacobi2d; do
    OnlyVec=1

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    for vsize_unpadded in 4 8 16 32 64 128 238; do
      vsize=$(($vsize_unpadded + 2))

      tempfile=`mktemp`

      # Clean, and then generate the correct data
      make -C apps/ clean

      mkdir -p apps/benchmarks/data
      ${PYTHON} apps/$kernel/script/gen_data.py $vsize $vsize $OnlyVec > apps/benchmarks/data/data.S
      config=${config} ENV_DEFINES="-D${kernel^^}=1" \
             make -C apps/ bin/benchmarks
      make -C hardware/ simv app=benchmarks > $tempfile || exit
      # Extract the performance
         cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
      ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}.benchmark

      if [ "$ci" == 0 ]; then
        # System with ideal dispatcher
        config=${config} ENV_DEFINES="-D${kernel^^}=1" \
               make -C apps/ bin/benchmarks.ideal
        touch -a hardware/build
        config=${config} make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
        # Extract the performance
           cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
      fi
    done
  done
}

#############
## DROPOUT ##
#############

dropout() {
  # Measure the runtime of the following kernels
  for kernel in dropout; do

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    for vsize in 4 8 16 32 64 128 256 512 1024 2048; do
      tempfile=`mktemp`

      # Clean
      make -C apps/ clean

      mkdir -p apps/benchmarks/data
      ${PYTHON} apps/$kernel/script/gen_data.py $vsize > apps/benchmarks/data/data.S

      # Standard System
      config=${config} ENV_DEFINES="-D${kernel^^}=1" \
             make -C apps/ bin/benchmarks
      make -C hardware/ simv app=benchmarks > $tempfile || exit
      # Extract the performance
         cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
      ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}.benchmark

      if [ "$ci" == 0 ]; then
        # System with ideal dispatcher
        config=${config} ENV_DEFINES="-D${kernel^^}=1" \
               make -C apps/ bin/benchmarks.ideal
        touch -a hardware/build
        config=${config} make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
        # Extract the performance
           cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
      fi
    done
  done
}

#########
## FFT ##
#########

fft() {
  # Measure the runtime of the following kernels
  for kernel in fft; do

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    dtype="float32"
    # Type should be in the format "floatXY"
    dbits=${dtype:5:2}

    # 2-lanes and vlen == 4096 cannot contain 256 float32 elements
    for vsize in 4 8 16 32 64 128 $(test $vlen -ge $(( 256 * ${dtype:5:2} )) && echo 256); do
      tempfile=`mktemp`

      # Clean, and then generate the correct matrix and filter
      make -C apps/ clean

      mkdir -p apps/benchmarks/data
      ${PYTHON} apps/$kernel/script/gen_data.py $vsize $dtype > apps/benchmarks/data/data.S

      config=${config} ENV_DEFINES="-D${kernel^^}=1 -DFFT_SAMPLES=${vsize}" \
             make -C apps/ bin/benchmarks
      make -C hardware/ simv app=benchmarks > $tempfile || exit
      # Extract the performance
      cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
      ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}.benchmark

      if [ "$ci" == 0 ]; then
        # System with ideal dispatcher
        config=${config} ENV_DEFINES="-D${kernel^^}=1 -DFFT_SAMPLES=${vsize}" \
               make -C apps/ bin/benchmarks.ideal
        touch -a hardware/build
        config=${config} make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
        # Extract the performance
           cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
      fi
    done
  done
}

#########
## DWT ##
#########

dwt() {
  # Measure the runtime of the following kernels
  for kernel in dwt; do

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    for vsize in 4 8 16 32 64 128 256 512; do

      tempfile=`mktemp`

      # Clean
      make -C apps/ clean

      mkdir -p apps/benchmarks/data
      ${PYTHON} apps/$kernel/script/gen_data.py $vsize > apps/benchmarks/data/data.S
      config=${config} ENV_DEFINES="-D${kernel^^}=1 -DSAMPLES=${vsize}" \
             make -C apps/ bin/benchmarks
      make -C hardware/ simv app=benchmarks > $tempfile || exit
      # Extract the performance
      cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
      ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}.benchmark

      if [ "$ci" == 0 ]; then
        # System with ideal dispatcher
        config=${config} ENV_DEFINES="-D${kernel^^}=1 -DSAMPLES=${vsize}" \
               make -C apps/ bin/benchmarks.ideal
        touch -a hardware/build
        config=${config} make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
        # Extract the performance
        cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
      fi
    done
  done
}

#########
## EXP ##
#########

exp() {
  # Measure the runtime of the following kernels
  for kernel in exp; do

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    for vsize in 8 16 32 64 128 256 512; do
      tempfile=`mktemp`

      # Clean
      make -C apps/ clean

      mkdir -p apps/benchmarks/data
      ${PYTHON} apps/$kernel/script/gen_data.py $vsize > apps/benchmarks/data/data.S
      ENV_DEFINES="-D${kernel^^}=1" \
             make -C apps/ bin/benchmarks
      make -C hardware/ simv app=benchmarks > $tempfile || exit
      # Extract the performance
      cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
      ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}.benchmark

      if [ "$ci" == 0 ]; then
        # System with ideal dispatcher
        ENV_DEFINES="-D${kernel^^}=1 -DSAMPLES=${vsize}" \
               make -C apps/ bin/benchmarks.ideal
        touch -a hardware/build
        make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
        # Extract the performance
        cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$vsize" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
      fi
    done
  done
}

#############
## SOFTMAX ##
#############

softmax() {
  # Measure the runtime of the following kernels
  for kernel in softmax; do

    # Log the performance results
    > ${kernel}_${nr_lanes}.benchmark
    > ${kernel}_${nr_lanes}_ideal.benchmark

    for insize in 8 16 32 64 128 256 512; do
      chsize=32
      tempfile=`mktemp`

      # Clean
      make -C apps/ clean

      mkdir -p apps/benchmarks/data
      ${PYTHON} apps/$kernel/script/gen_data.py $chsize $insize > apps/benchmarks/data/data.S
      ENV_DEFINES="-D${kernel^^}=1" \
             make -C apps/ bin/benchmarks
      make -C hardware/ simv app=benchmarks > $tempfile || exit
      # Extract the performance
      cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
      ./scripts/performance.py $kernel "$chsize $insize" $cycles >> ${kernel}_${nr_lanes}.benchmark

      if [ "$ci" == 0 ]; then
        # System with ideal dispatcher
        ENV_DEFINES="-D${kernel^^}=1" \
               make -C apps/ bin/benchmarks.ideal
        touch -a hardware/build
        make -C hardware/ -B simc app=benchmarks ideal_dispatcher=1 > $tempfile || exit
        # Extract the performance
        cycles=$(cat $tempfile | grep "\[cycles\]" | cut -d: -f2)
        ./scripts/performance.py $kernel "$chsize $insize" $cycles >> ${kernel}_${nr_lanes}_ideal.benchmark
      fi
    done
  done
}

case $1 in
  "matmul")
    matmul
    ;;

  "conv2d")
    conv2d
    ;;

  "conv3d")
    conv3d
    ;;

  "jacobi2d")
    jacobi2d
    ;;

  "dropout")
    dropout
    ;;

  "fft")
    fft
    ;;

  "dwt")
    dwt
    ;;

  "exp")
    exp
    ;;

  "softmax")
    softmax
    ;;

  *)
    echo "Benchmarking all the apps."
    matmul
    conv2d
    conv3d
    jacobi2d
    dropout
    fft
    dwt
    exp
    softmax
    ;;
esac
