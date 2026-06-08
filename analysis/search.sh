#!/usr/bin/env bash
# search.sh — parameter sweep runner
#
# Configure ANALYSIS_MODE and the parameter lists below, then run:
#   bash search.sh
#
# Comment in/out lists to control what gets swept.
# Anything without a list falls back to its FIXED_* default.
# Results go to sweep_results.csv (per run) and sweep_summary.csv (per config).

set -euo pipefail # exit on failure

BINARY="../build/lidar_sim"
LOG_BASE="logs"
RESULTS_CSV="sweep_results.csv"
SUMMARY_CSV="sweep_summary.csv"
CONFIGS_TXT="sweep_configs.txt"
MAKE_DIR=".."

TIMEOUT_DURATION=60 # seconds
REPEATS=3

# Analysis Mode:
# For ekf: uses preset waypoints + full obstacle scene.
# Set ODOM_NOISE=0 to disable odometry noise for clean EKF testing.
ANALYSIS_MODE="mppi"
ODOM_NOISE=0 # 1 = enabled, 0 = disabled

# Fixed default values if not tested
FIXED_SPEED_NOISE="0.05"
FIXED_ANGULAR_NOISE_DEG="2.0"
FIXED_LIDAR_NOISE="0.05"
FIXED_ANGLE_NOISE_DEG="1.5"
FIXED_LAMBDA="4.0"
FIXED_SAMPLES="256"
FIXED_HORIZON="80"
FIXED_W_HEADING="8.0"
FIXED_W_SPEED="6.0"
FIXED_SIGMA_STEER="1.2"

# Hyperparameter list
# Comment out to hold parameter fixed with default value

# MPPI Weights
W_HEADING_LIST="4.0 8.0 12.0"
W_SPEED_LIST="2.0 6.0 10.0"

# MPPI Architecture
# MPPI_SAMPLES_LIST="128 256"
# MPPI_HORIZON_LIST="60 80"

# MPPI Exploration vs Exploitation
# SIGMA_STEER_LIST="0.8 1.2 1.5"
# MPPI_LAMBDA_LIST="1.0 2.5 4.0 7.0 10.0"

# EKF Process Noise
# SPEED_NOISE_LIST="0.02 0.05 0.10"
# ANGULAR_NOISE_LIST="0.5 2.0 4.0"

# EKF Measurement Noise
# LIDAR_NOISE_LIST="0.02 0.05 0.10"
# ANGLE_NOISE_LIST="0.75 1.5 3.0"


deg2rad_cfloat() { # convert degrees to a C float literal
    echo "$1" | awk '{ printf "%.6ff\n", $1 * 3.14159265358979 / 180 }'
}

awk_meanstd() {
    echo "$1" | awk '{
        n = split($0, a)
        s = 0; for (i=1;i<=n;i++) s += a[i]
        mean = s / n
        v = 0; for (i=1;i<=n;i++) v += (a[i]-mean)^2
        std = (n > 1) ? sqrt(v/(n-1)) : 0
        printf "%.3f %.3f\n", mean, std
    }'
}

awk_mean() {
    echo "$1" | awk '{
        n = split($0, a)
        s = 0; for (i=1;i<=n;i++) s += a[i]
        printf "%.4f\n", s / n
    }'
}

# CSV Headers

echo "run_id,config_id,repeat,ANALYSIS_MODE,speed_noise,angular_noise_deg,lidar_noise,angle_noise_deg,mppi_lambda,mppi_samples,mppi_horizon,w_heading,w_speed,sigma_steer,odom_noise,completion_time_s" > "$RESULTS_CSV"
echo "config_id,ANALYSIS_MODE,speed_noise,angular_noise_deg,lidar_noise,angle_noise_deg,mppi_lambda,mppi_samples,mppi_horizon,w_heading,w_speed,sigma_steer,odom_noise,repeats,dnf_count,mean_completion_time_s,std_completion_time_s" > "$SUMMARY_CSV"

mkdir -p "$LOG_BASE"

RUN_ID=0
CONFIG_ID=0

# Core
run_config() {
    local speed_noise="$1"
    local angular_noise_deg="$2"
    local lidar_noise="$3"
    local angle_noise_deg="$4"
    local mppi_lambda="$5"
    local mppi_samples="$6"
    local mppi_horizon="$7"
    local w_heading="$8"
    local w_speed="$9"
    local sigma_steer="${10}"

    CONFIG_ID=$((CONFIG_ID + 1))
    local config_id="$CONFIG_ID"
    local angular_noise_rad; angular_noise_rad=$(deg2rad_cfloat "$angular_noise_deg")
    local angle_noise_rad; angle_noise_rad=$(deg2rad_cfloat "$angle_noise_deg")

    local noise_flag=""
    if [ "$ODOM_NOISE" = "0" ]; then
        noise_flag="-DDISABLE_ODOM_NOISE"
    fi

    # Write config entry
    {
        echo "config_id=${config_id}"
        echo "  R_t: speed_noise=${speed_noise} | angular_noise=${angular_noise_deg}deg"
        echo "  Q_t: lidar_noise=${lidar_noise} | angle_noise=${angle_noise_deg}deg"
        echo "  mppi: lambda=${mppi_lambda} | samples=${mppi_samples} | horizon=${mppi_horizon}"
        echo "  mppi: w_heading=${w_heading} | w_speed=${w_speed} | sigma_steer=${sigma_steer}"
        echo ""
    } >> "$CONFIGS_TXT"

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Config ${config_id} | mode=${ANALYSIS_MODE} | ${REPEATS} repeats | odom_noise=${ODOM_NOISE}"
    echo "  R_t: speed=${speed_noise}  angular=${angular_noise_deg}deg"
    echo "  Q_t: lidar=${lidar_noise}  angle=${angle_noise_deg}deg"
    echo "  lambda=${mppi_lambda}  samples=${mppi_samples}  horizon=${mppi_horizon}"
    echo "  W_HEADING=${w_heading}  W_SPEED=${w_speed}  sigma_steer=${sigma_steer}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    echo "[BUILD] Compiling config ${config_id}..."
    make -C "$MAKE_DIR" clean -s
    make -C "$MAKE_DIR" \
        EXTRAFLAGS="${noise_flag} \
            -DEKF_DEFAULT_SPEED_NOISE=${speed_noise}f \
            -DEKF_DEFAULT_ANGULAR_NOISE=${angular_noise_rad} \
            -DEKF_DEFAULT_LIDAR_NOISE=${lidar_noise}f \
            -DEKF_DEFAULT_ANGLE_NOISE=${angle_noise_rad} \
            -DMPPI_LAMBDA=${mppi_lambda}f \
            -DMPPI_SAMPLES=${mppi_samples} \
            -DMPPI_HORIZON=${mppi_horizon} \
            -DW_HEADING=${w_heading}f \
            -DW_SPEED=${w_speed}f \
            -DMPPI_SIGMA_STEER=${sigma_steer}f \
            -DMETRICS_RUN_ID=${config_id}" \
        2>&1 | tail -3

    local dnf_count=0
    local completion_times=""

    for repeat in $(seq 1 "$REPEATS"); do
        RUN_ID=$((RUN_ID + 1))
        local run_id="$RUN_ID"

        echo "[RUN] Repeat ${repeat}/${REPEATS} (run_id=${run_id})..."

        # Declare first, then assign — avoids set -e swallowing non-zero exit codes
        local sim_stderr
        sim_stderr=$(gtimeout "$TIMEOUT_DURATION" "$BINARY" \
            --sweep-mode "$ANALYSIS_MODE" --duration "$TIMEOUT_DURATION" --headless 2>&1 >/dev/null) || true

        local completion_time="DNF"
        if echo "$sim_stderr" | grep -q "\[run\] complete"; then
            completion_time=$(echo "$sim_stderr" \
                | grep "\[run\] complete" \
                | sed 's/.*complete in //' \
                | sed 's/s//')
        fi
        echo "  completion=${completion_time}s"

        # Move log files into LOGBASE
        for f in icp_matches icp_iters ekf_updates mppi_steps rover_ground_truth; do
            local src="${LOG_BASE}/${f}_${config_id}.csv"
            local dst="${LOG_BASE}/${f}_${run_id}.csv"
            [ -f "$src" ] && mv "$src" "$dst"
        done

        if [ "$completion_time" = "DNF" ]; then
            dnf_count=$((dnf_count + 1))
            echo "${run_id},${config_id},${repeat},${ANALYSIS_MODE},${speed_noise},${angular_noise_deg},${lidar_noise},${angle_noise_deg},${mppi_lambda},${mppi_samples},${mppi_horizon},${w_heading},${w_speed},${sigma_steer},${ODOM_NOISE},DNF" >> "$RESULTS_CSV"
            continue
        fi

        echo "${run_id},${config_id},${repeat},${ANALYSIS_MODE},${speed_noise},${angular_noise_deg},${lidar_noise},${angle_noise_deg},${mppi_lambda},${mppi_samples},${mppi_horizon},${w_heading},${w_speed},${sigma_steer},${ODOM_NOISE},${completion_time}" >> "$RESULTS_CSV"

        completion_times="${completion_times} ${completion_time}"
    done

    # Summarise repeats
    local mean_t="DNF" std_t="DNF"

    if [ -n "${completion_times// /}" ]; then # check not all DNF
        read -r mean_t std_t < <(awk_meanstd "$completion_times")
    fi

    echo "${config_id},${ANALYSIS_MODE},${speed_noise},${angular_noise_deg},${lidar_noise},${angle_noise_deg},${mppi_lambda},${mppi_samples},${mppi_horizon},${w_heading},${w_speed},${sigma_steer},${ODOM_NOISE},${REPEATS},${dnf_count},${mean_t},${std_t}" >> "$SUMMARY_CSV"

    echo "[DONE] config=${config_id} mean_time=${mean_t}s std=${std_t}s dnf=${dnf_count}/${REPEATS}"
}

# Resolve parameter lists (fall back to fixed defaults)

_speed_noises="${SPEED_NOISE_LIST:-$FIXED_SPEED_NOISE}"
_angular_noises="${ANGULAR_NOISE_LIST:-$FIXED_ANGULAR_NOISE_DEG}"
_lidar_noises="${LIDAR_NOISE_LIST:-$FIXED_LIDAR_NOISE}"
_angle_noises="${ANGLE_NOISE_LIST:-$FIXED_ANGLE_NOISE_DEG}"
_lambdas="${MPPI_LAMBDA_LIST:-$FIXED_LAMBDA}"
_samples="${MPPI_SAMPLES_LIST:-$FIXED_SAMPLES}"
_horizons="${MPPI_HORIZON_LIST:-$FIXED_HORIZON}"
_w_headings="${W_HEADING_LIST:-$FIXED_W_HEADING}"
_w_speeds="${W_SPEED_LIST:-$FIXED_W_SPEED}"
_sigmas="${SIGMA_STEER_LIST:-$FIXED_SIGMA_STEER}"


# Run
for sn in $_speed_noises; do
for an in $_angular_noises; do
for ln in $_lidar_noises; do
for aln in $_angle_noises; do
for lam in $_lambdas; do
for samp in $_samples; do
for horiz in $_horizons; do
for wh in $_w_headings; do
for ws in $_w_speeds; do
for sig in $_sigmas; do
    run_config "$sn" "$an" "$ln" "$aln" "$lam" "$samp" "$horiz" "$wh" "$ws" "$sig"
done; done; done; done; done
done; done; done; done; done