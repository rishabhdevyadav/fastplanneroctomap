# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rishabh/catkin_ws/src/PX4-Autopilot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rishabh/catkin_ws/build/px4

# Utility rule file for shellcheck.

# Include the progress variables for this target.
include ROMFS/CMakeFiles/shellcheck.dir/progress.make

ROMFS/CMakeFiles/shellcheck: etc/init.d/rc.autostart
	cd /home/rishabh/catkin_ws/build/px4/etc && /usr/bin/shellcheck --shell=sh --exclude=SC1090 --exclude=SC1091 --exclude=SC2121 --exclude=SC2086 --exclude=SC2166 --exclude=SC2154 --exclude=SC2164 --exclude=SC2169 --exclude=SC2039 --exclude=SC2181 `find /home/rishabh/catkin_ws/build/px4/etc/init.d -type f`

etc/init.d/rc.serial: ROMFS/romfs_extract.stamp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rishabh/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "ROMFS: copying, generating airframes"
	cd /home/rishabh/catkin_ws/build/px4/ROMFS && /usr/bin/python2 /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/px_process_airframes.py --airframes-path /home/rishabh/catkin_ws/build/px4/etc/init.d --start-script /home/rishabh/catkin_ws/build/px4/etc/init.d/rc.autostart --board px4_sitl
	cd /home/rishabh/catkin_ws/build/px4/ROMFS && /usr/bin/python2 /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/serial/generate_config.py --rc-dir /home/rishabh/catkin_ws/build/px4/etc/init.d --serial-ports --config-files /home/rishabh/catkin_ws/src/PX4-Autopilot/src/lib/battery/module.yaml /home/rishabh/catkin_ws/src/PX4-Autopilot/src/drivers/gps/module.yaml /home/rishabh/catkin_ws/src/PX4-Autopilot/src/modules/mavlink/module.yaml
	cd /home/rishabh/catkin_ws/build/px4/ROMFS && /usr/bin/cmake -E touch romfs_copy.stamp

etc/init.d/rc.autostart: etc/init.d/rc.serial
	@$(CMAKE_COMMAND) -E touch_nocreate etc/init.d/rc.autostart

etc/init.d/rc.autostart.post: etc/init.d/rc.serial
	@$(CMAKE_COMMAND) -E touch_nocreate etc/init.d/rc.autostart.post

ROMFS/romfs_copy.stamp: etc/init.d/rc.serial
	@$(CMAKE_COMMAND) -E touch_nocreate ROMFS/romfs_copy.stamp

ROMFS/romfs_extract.stamp: romfs_files.tar
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rishabh/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating romfs_extract.stamp"
	cd /home/rishabh/catkin_ws/build/px4/etc && /usr/bin/cmake -E tar xf /home/rishabh/catkin_ws/build/px4/romfs_files.tar
	cd /home/rishabh/catkin_ws/build/px4/etc && /usr/bin/cmake -E touch /home/rishabh/catkin_ws/build/px4/ROMFS/romfs_extract.stamp

romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/1000_rc_fw_easystar.hil
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/1001_rc_quad_x.hil
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/1002_standard_vtol.hil
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/1100_rc_quad_x_sih.hil
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/2100_standard_plane
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/2105_maja
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/2106_albatross
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/2200_mini_talon
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/2507_cloudship
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3000_generic_wing
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3030_io_camflyer
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3031_phantom
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3032_skywalker_x5
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3033_wingwing
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3034_fx79
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3035_viper
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3036_pigeon
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3037_parrot_disco_mod
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/3100_tbs_caipirinha
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4001_quad_x
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4003_qavr5
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4009_qav250
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4010_dji_f330
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4011_dji_f450
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4014_s500
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4015_holybro_s500
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4016_holybro_px4vision
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4017_nxp_hovergames
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4020_hk_micro_pcb
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4030_3dr_solo
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4031_3dr_quad
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4040_reaper
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4041_beta75x
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4050_generic_250
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4051_s250aq
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4052_holybro_qav250
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4053_holybro_kopis2
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4060_dji_matrice_100
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4070_aerofc
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4071_ifo
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4072_draco
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4073_ifo-s
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4080_zmr250
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4090_nanomind
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4100_tiltquadrotor
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4250_teal
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4500_clover4
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/4900_crazyflie
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/5001_quad_+
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/6001_hexa_x
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/6002_draco_r
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/7001_hexa_+
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/8001_octo_x
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/9001_octo_+
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/10015_tbs_discovery
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/10016_3dr_iris
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/10017_steadidrone_qu4d
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/10018_tbs_endurance
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/11001_hexa_cox
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/12001_octo_cox
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/12002_steadidrone_mavrik
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13000_generic_vtol_standard
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13001_caipirinha_vtol
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13002_firefly6
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13003_quad_tailsitter
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13004_quad+_tailsitter
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13005_vtol_AAERT_quad
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13006_vtol_standard_delta
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13007_vtol_AAVVT_quad
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13008_QuadRanger
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13009_vtol_spt_ranger
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13010_claire
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13012_convergence
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13013_deltaquad
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13014_vtol_babyshark
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13050_generic_vtol_octo
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/13200_generic_vtol_tailsitter
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/14001_tri_y_yaw+
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/14002_tri_y_yaw-
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/15001_coax_heli
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/16001_helicopter
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/17002_TF-AutoG2
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/24001_dodeca_cox
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/50000_generic_ground_vehicle
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/50001_axialracing_ax10
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/50002_traxxas_stampede_2wd
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/50003_aion_robotics_r1_rover
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/50004_nxpcup_car_dfrobot_gpx
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/60000_uuv_generic
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/60001_uuv_hippocampus
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/60002_uuv_bluerov2_heavy
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.airship_apps
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.airship_defaults
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.boat_defaults
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.fw_apps
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.fw_defaults
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.interface
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.io
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.logging
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.mc_apps
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.mc_defaults
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rcS
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.sensors
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.thermal_cal
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.rover_apps
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.rover_defaults
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.uuv_apps
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.uuv_defaults
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.vehicle_setup
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.vtol_apps
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.vtol_defaults
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/AAERTWF.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/AAVVTWFF.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/AAVVTWFF_vtail.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/AERT.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/AETRFG.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/babyshark.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/blade130.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/caipi.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/CCPM.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/claire.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/claire.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/cloudship.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/coax.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/delta.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/deltaquad.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/dodeca_bottom_cox.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/dodeca_top_cox.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/firefly6.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/firefly6.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/fw_generic_wing.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/FX79.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/generic_diff_rover.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/hexa_cox.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/hexa_+.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/hexa_x.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/IO_pass.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/mount.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/mount_legs.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/octo_cox.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/octo_cox_w.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/octo_+.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/octo_x.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/pass.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/phantom.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_dc.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_h.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_+.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_s250aq.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_+_vtol.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_w.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_x_cw.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_x.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/quad_x_vtol.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/rover_diff_and_servo.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/rover_generic.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/stampede.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/standard_vtol_hitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/TF-AutoG2.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/tilt_quad.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/tilt_quad.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/tri_y_yaw+.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/tri_y_yaw-.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/uuv_x.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/vectored6dof.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/Viper.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/vtol_AAERT.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/vtol_AAVVT.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/vtol_convergence.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/vtol_delta.aux.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/vtol_tailsitter_duo.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/wingwing.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/autogyro_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/boat_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/delta_wing_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/plane_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/quad_x_vtol.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/rover_ackermann_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/rover_diff_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/standard_vtol_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/tiltrotor_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/uuv_x_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/vectored6dof_sitl.main.mix
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10016_iris
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10020_if750a
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10030_px4vision
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1010_iris_opt_flow
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1010_iris_opt_flow.post
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1011_iris_irlock
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1012_iris_rplidar
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1013_iris_vision
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1013_iris_vision.post
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1014_solo
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1015_iris_obs_avoid
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1015_iris_obs_avoid.post
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1016_iris_rtps
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1016_iris_rtps.post
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1017_iris_opt_flow_mockup
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1018_iris_vision_velocity
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1019_iris_dual_gps
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1020_uuv_generic
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1021_uuv_hippocampus
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1022_uuv_bluerov2_heavy
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1030_plane
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1031_plane_cam
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1032_plane_catapult
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1033_plane_lidar
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1033_rascal
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1034_rascal-electric
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1035_techpod
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1036_malolo
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1040_standard_vtol
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1041_tailsitter
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1042_tiltrotor
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1060_rover
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1061_r1_rover
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1062_tf-r1
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1070_boat
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/3010_quadrotor_x
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/3011_hexarotor_x
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/17001_tf-g1
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/2507_cloudship
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480.post
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rc.replay
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/serial/rc.serial.jinja
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/serial/rc.serial_port.jinja
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/serial/serial_params.c.jinja
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/src/lib/battery/module.yaml
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/src/drivers/gps/module.yaml
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/src/modules/mavlink/module.yaml
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d/CMakeLists.txt
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers/CMakeLists.txt
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/mixers-sitl/CMakeLists.txt
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/CMakeLists.txt
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/px_process_airframes.py
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/px4airframes/markdownout.py
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/px4airframes/rcout.py
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/px4airframes/srcparser.py
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/px4airframes/srcscanner.py
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/px4airframes/xmlout.py
romfs_files.tar: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/serial/generate_config.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rishabh/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating ../romfs_files.tar"
	cd /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS/px4fmu_common && /usr/bin/cmake -E tar cf /home/rishabh/catkin_ws/build/px4/romfs_files.tar init.d/airframes/1000_rc_fw_easystar.hil init.d/airframes/1001_rc_quad_x.hil init.d/airframes/1002_standard_vtol.hil init.d/airframes/1100_rc_quad_x_sih.hil init.d/airframes/2100_standard_plane init.d/airframes/2105_maja init.d/airframes/2106_albatross init.d/airframes/2200_mini_talon init.d/airframes/2507_cloudship init.d/airframes/3000_generic_wing init.d/airframes/3030_io_camflyer init.d/airframes/3031_phantom init.d/airframes/3032_skywalker_x5 init.d/airframes/3033_wingwing init.d/airframes/3034_fx79 init.d/airframes/3035_viper init.d/airframes/3036_pigeon init.d/airframes/3037_parrot_disco_mod init.d/airframes/3100_tbs_caipirinha init.d/airframes/4001_quad_x init.d/airframes/4003_qavr5 init.d/airframes/4009_qav250 init.d/airframes/4010_dji_f330 init.d/airframes/4011_dji_f450 init.d/airframes/4014_s500 init.d/airframes/4015_holybro_s500 init.d/airframes/4016_holybro_px4vision init.d/airframes/4017_nxp_hovergames init.d/airframes/4020_hk_micro_pcb init.d/airframes/4030_3dr_solo init.d/airframes/4031_3dr_quad init.d/airframes/4040_reaper init.d/airframes/4041_beta75x init.d/airframes/4050_generic_250 init.d/airframes/4051_s250aq init.d/airframes/4052_holybro_qav250 init.d/airframes/4053_holybro_kopis2 init.d/airframes/4060_dji_matrice_100 init.d/airframes/4070_aerofc init.d/airframes/4071_ifo init.d/airframes/4072_draco init.d/airframes/4073_ifo-s init.d/airframes/4080_zmr250 init.d/airframes/4090_nanomind init.d/airframes/4100_tiltquadrotor init.d/airframes/4250_teal init.d/airframes/4500_clover4 init.d/airframes/4900_crazyflie init.d/airframes/5001_quad_+ init.d/airframes/6001_hexa_x init.d/airframes/6002_draco_r init.d/airframes/7001_hexa_+ init.d/airframes/8001_octo_x init.d/airframes/9001_octo_+ init.d/airframes/10015_tbs_discovery init.d/airframes/10016_3dr_iris init.d/airframes/10017_steadidrone_qu4d init.d/airframes/10018_tbs_endurance init.d/airframes/11001_hexa_cox init.d/airframes/12001_octo_cox init.d/airframes/12002_steadidrone_mavrik init.d/airframes/13000_generic_vtol_standard init.d/airframes/13001_caipirinha_vtol init.d/airframes/13002_firefly6 init.d/airframes/13003_quad_tailsitter init.d/airframes/13004_quad+_tailsitter init.d/airframes/13005_vtol_AAERT_quad init.d/airframes/13006_vtol_standard_delta init.d/airframes/13007_vtol_AAVVT_quad init.d/airframes/13008_QuadRanger init.d/airframes/13009_vtol_spt_ranger init.d/airframes/13010_claire init.d/airframes/13012_convergence init.d/airframes/13013_deltaquad init.d/airframes/13014_vtol_babyshark init.d/airframes/13050_generic_vtol_octo init.d/airframes/13200_generic_vtol_tailsitter init.d/airframes/14001_tri_y_yaw+ init.d/airframes/14002_tri_y_yaw- init.d/airframes/15001_coax_heli init.d/airframes/16001_helicopter init.d/airframes/17002_TF-AutoG2 init.d/airframes/24001_dodeca_cox init.d/airframes/50000_generic_ground_vehicle init.d/airframes/50001_axialracing_ax10 init.d/airframes/50002_traxxas_stampede_2wd init.d/airframes/50003_aion_robotics_r1_rover init.d/airframes/50004_nxpcup_car_dfrobot_gpx init.d/airframes/60000_uuv_generic init.d/airframes/60001_uuv_hippocampus init.d/airframes/60002_uuv_bluerov2_heavy init.d/rc.airship_apps init.d/rc.airship_defaults init.d/rc.boat_defaults init.d/rc.fw_apps init.d/rc.fw_defaults init.d/rc.interface init.d/rc.io init.d/rc.logging init.d/rc.mc_apps init.d/rc.mc_defaults init.d/rcS init.d/rc.sensors init.d/rc.thermal_cal init.d/rc.rover_apps init.d/rc.rover_defaults init.d/rc.uuv_apps init.d/rc.uuv_defaults init.d/rc.vehicle_setup init.d/rc.vtol_apps init.d/rc.vtol_defaults mixers/AAERTWF.main.mix mixers/AAVVTWFF.main.mix mixers/AAVVTWFF_vtail.main.mix mixers/AERT.main.mix mixers/AETRFG.main.mix mixers/babyshark.main.mix mixers/blade130.main.mix mixers/caipi.main.mix mixers/CCPM.main.mix mixers/claire.aux.mix mixers/claire.main.mix mixers/cloudship.main.mix mixers/coax.main.mix mixers/delta.main.mix mixers/deltaquad.main.mix mixers/dodeca_bottom_cox.aux.mix mixers/dodeca_top_cox.main.mix mixers/firefly6.aux.mix mixers/firefly6.main.mix mixers/fw_generic_wing.main.mix mixers/FX79.main.mix mixers/generic_diff_rover.main.mix mixers/hexa_cox.main.mix mixers/hexa_+.main.mix mixers/hexa_x.main.mix mixers/IO_pass.main.mix mixers/mount.aux.mix mixers/mount_legs.aux.mix mixers/octo_cox.main.mix mixers/octo_cox_w.main.mix mixers/octo_+.main.mix mixers/octo_x.main.mix mixers/pass.aux.mix mixers/phantom.main.mix mixers/quad_dc.main.mix mixers/quad_h.main.mix mixers/quad_+.main.mix mixers/quad_s250aq.main.mix mixers/quad_+_vtol.main.mix mixers/quad_w.main.mix mixers/quad_x_cw.main.mix mixers/quad_x.main.mix mixers/quad_x_vtol.main.mix mixers/rover_diff_and_servo.main.mix mixers/rover_generic.main.mix mixers/stampede.main.mix mixers/standard_vtol_hitl.main.mix mixers/TF-AutoG2.main.mix mixers/tilt_quad.aux.mix mixers/tilt_quad.main.mix mixers/tri_y_yaw+.main.mix mixers/tri_y_yaw-.main.mix mixers/uuv_x.main.mix mixers/vectored6dof.main.mix mixers/Viper.main.mix mixers/vtol_AAERT.aux.mix mixers/vtol_AAVVT.aux.mix mixers/vtol_convergence.main.mix mixers/vtol_delta.aux.mix mixers/vtol_tailsitter_duo.main.mix mixers/wingwing.main.mix mixers-sitl/autogyro_sitl.main.mix mixers-sitl/boat_sitl.main.mix mixers-sitl/delta_wing_sitl.main.mix mixers-sitl/plane_sitl.main.mix mixers-sitl/quad_x_vtol.main.mix mixers-sitl/rover_ackermann_sitl.main.mix mixers-sitl/rover_diff_sitl.main.mix mixers-sitl/standard_vtol_sitl.main.mix mixers-sitl/tiltrotor_sitl.main.mix mixers-sitl/uuv_x_sitl.main.mix mixers-sitl/vectored6dof_sitl.main.mix init.d-posix/airframes/10016_iris init.d-posix/airframes/10020_if750a init.d-posix/airframes/10030_px4vision init.d-posix/airframes/1010_iris_opt_flow init.d-posix/airframes/1010_iris_opt_flow.post init.d-posix/airframes/1011_iris_irlock init.d-posix/airframes/1012_iris_rplidar init.d-posix/airframes/1013_iris_vision init.d-posix/airframes/1013_iris_vision.post init.d-posix/airframes/1014_solo init.d-posix/airframes/1015_iris_obs_avoid init.d-posix/airframes/1015_iris_obs_avoid.post init.d-posix/airframes/1016_iris_rtps init.d-posix/airframes/1016_iris_rtps.post init.d-posix/airframes/1017_iris_opt_flow_mockup init.d-posix/airframes/1018_iris_vision_velocity init.d-posix/airframes/1019_iris_dual_gps init.d-posix/airframes/1020_uuv_generic init.d-posix/airframes/1021_uuv_hippocampus init.d-posix/airframes/1022_uuv_bluerov2_heavy init.d-posix/airframes/1030_plane init.d-posix/airframes/1031_plane_cam init.d-posix/airframes/1032_plane_catapult init.d-posix/airframes/1033_plane_lidar init.d-posix/airframes/1033_rascal init.d-posix/airframes/1034_rascal-electric init.d-posix/airframes/1035_techpod init.d-posix/airframes/1036_malolo init.d-posix/airframes/1040_standard_vtol init.d-posix/airframes/1041_tailsitter init.d-posix/airframes/1042_tiltrotor init.d-posix/airframes/1060_rover init.d-posix/airframes/1061_r1_rover init.d-posix/airframes/1062_tf-r1 init.d-posix/airframes/1070_boat init.d-posix/airframes/3010_quadrotor_x init.d-posix/airframes/3011_hexarotor_x init.d-posix/airframes/17001_tf-g1 init.d-posix/airframes/2507_cloudship init.d-posix/airframes/6011_typhoon_h480 init.d-posix/airframes/6011_typhoon_h480.post init.d-posix/rc.replay init.d-posix/rcS

shellcheck: ROMFS/CMakeFiles/shellcheck
shellcheck: etc/init.d/rc.serial
shellcheck: etc/init.d/rc.autostart
shellcheck: etc/init.d/rc.autostart.post
shellcheck: ROMFS/romfs_copy.stamp
shellcheck: ROMFS/romfs_extract.stamp
shellcheck: romfs_files.tar
shellcheck: ROMFS/CMakeFiles/shellcheck.dir/build.make

.PHONY : shellcheck

# Rule to build all files generated by this target.
ROMFS/CMakeFiles/shellcheck.dir/build: shellcheck

.PHONY : ROMFS/CMakeFiles/shellcheck.dir/build

ROMFS/CMakeFiles/shellcheck.dir/clean:
	cd /home/rishabh/catkin_ws/build/px4/ROMFS && $(CMAKE_COMMAND) -P CMakeFiles/shellcheck.dir/cmake_clean.cmake
.PHONY : ROMFS/CMakeFiles/shellcheck.dir/clean

ROMFS/CMakeFiles/shellcheck.dir/depend:
	cd /home/rishabh/catkin_ws/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rishabh/catkin_ws/src/PX4-Autopilot /home/rishabh/catkin_ws/src/PX4-Autopilot/ROMFS /home/rishabh/catkin_ws/build/px4 /home/rishabh/catkin_ws/build/px4/ROMFS /home/rishabh/catkin_ws/build/px4/ROMFS/CMakeFiles/shellcheck.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ROMFS/CMakeFiles/shellcheck.dir/depend
