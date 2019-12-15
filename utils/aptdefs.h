#ifndef _aptdefs_h__
#define _aptdefs_h__

enum {
	// File format record codes
	apt_airport 		= 1,
	apt_rwy_old 		= 10,			// Legacy runway/taxiway record from 810 and earlier.
	apt_tower_loc 		= 14,
	apt_startup_loc 	= 15,
	apt_seaport 		= 16,
	apt_heliport 		= 17,
	apt_beacon 			= 18,
	apt_windsock 		= 19,
	apt_freq_awos 		= 50,
	apt_freq_ctaf 		= 51,
	apt_freq_del 		= 52,
	apt_freq_gnd 		= 53,
	apt_freq_twr 		= 54,
	apt_freq_app 		= 55,
	apt_freq_dep 		= 56,
	apt_done 			= 99,
	// These records are new with X-Plane 850!
	apt_sign 			= 20,
	apt_papi 			= 21,

	apt_rwy_new 		= 100,			// These replace the old type 10 record.
	apt_sea_new 		= 101,
	apt_heli_new 		= 102,
	apt_taxi_new 		= 110,
	apt_free_chain		= 120,
	apt_boundary 		= 130,

	apt_lin_seg 		= 111,
	apt_lin_crv 		= 112,
	apt_rng_seg 		= 113,
	apt_rng_crv 		= 114,
	apt_end_seg		 	= 115,
	apt_end_crv 		= 116,

	apt_flow_def		= 1000,			// 1000 <traffic flow name, must be unique to the ICAO airport>
	apt_flow_wind		= 1001,			// 1001 <metar icao> <wind dir min> <wind dir max> <wind max speed>
	apt_flow_ceil		= 1002,			// 1002 <metar icao> <ceiling minimum>
	apt_flow_vis		= 1003,			// 1003 <metar icao> <vis minimum>
	apt_flow_time		= 1004,			// 1004 <zulu time start> <zulu time end>

	apt_freq_awos_1k	= 1050,         // XP 1130 1kHz resolution freq
	apt_freq_ctaf_1k	= 1051,
	apt_freq_del_1k 	= 1052,
	apt_freq_gnd_1k		= 1053,
	apt_freq_twr_1k		= 1054,
	apt_freq_app_1k		= 1055,
	apt_freq_dep_1k		= 1056,
	
	apt_flow_rwy_rule	= 1100,
	apt_flow_pattern	= 1101,
	apt_flow_rwy_rule1k	= 1110,         // XP 1130 1kHz resolution freq
	
	apt_taxi_header		= 1200,			// 1200 <name>
	apt_taxi_node		= 1201,			// 1201 <lat> <lon> <type> <id, 0 based sequence, ascending> <name>
	apt_taxi_edge		= 1202,			// 1202 <src> <dst> <oneway flag> <runway flag/taxi width> <name>
	apt_taxi_shape		= 1203,			// 1203 <lat> <lon>
	apt_taxi_active		= 1204,			// 1204 type|flags runway,list
#if HAS_CURVED_ATC_ROUTE
	apt_taxi_control	= 1205,			// 1205 <lat> <lon>
#else
	apt_taxi_control	= 1205,			// 1205 just gracefully ignore these for now. Some are already in the apt.dat
#endif
	apt_taxi_truck_edge = 1206,			// 1206 <src> <dst> <oneway flag> <name>

	apt_startup_loc_new	= 1300,			// 1300 lat lon heading misc|gate|tie_down|hangar traffic name
	apt_startup_loc_extended = 1301,	// 1301 size opertaions_type airline_list
	apt_meta_data = 1302,				// 1302 <key> <value>
	
	apt_truck_parking	= 1400,			// 1400 lat lon heading type cars name
	apt_truck_destination = 1401,		// 1401 lat lon heading type|type|type... name
	
	// Surface codes
	apt_surf_none		= 0,
	apt_surf_asphalt,
	apt_surf_concrete,
	apt_surf_grass,
	apt_surf_dirt,
	apt_surf_gravel,
	apt_surf_asphalt_heli,			// these are 810 only
	apt_surf_concrete_heli,
	apt_surf_grass_heli,
	apt_surf_dirt_heli,
	apt_surf_asphalt_line,
	apt_surf_concrete_line,
	apt_surf_dry_lake,				// all versions
	apt_surf_water,
	apt_surf_ice,					// 850 only
	apt_surf_transparent,

	// Light Fixture Codes (850)
	apt_gls_vasi			= 1,
	apt_gls_papi_left,
	apt_gls_papi_right,
	apt_gls_papi_20,
	apt_gls_vasi_tricolor,
	apt_gls_wigwag,
	// VASI codes (810)
	apt_gls_none_810 = 1,
	apt_gls_vasi_810,
	apt_gls_papi_810,
	apt_gls_papi20_810,

	// Edge Light Codes (850)
	apt_edge_none = 0,
	apt_edge_LIRL,
	apt_edge_MIRL,
	apt_edge_HIRL,
	apt_heli_edge_none = 0,
	apt_heli_edge_yellow,
	// REIL Codes (850)
	apt_reil_none = 0,
	apt_reil_omni,
	apt_reil_uni,
	// Edge Light Codes (810)
	apt_edge_none_810 = 1,
	apt_edge_MIRL_810,
	apt_edge_REIL_810,
	apt_edge_CLL_810,
	apt_edge_TDZL_810,
	apt_edge_taxiway_810,

	// Approach Lights (850)
	apt_app_none = 0,
	apt_app_ALSFI,
	apt_app_ALSFII,
	apt_app_CALVERTI,
	apt_app_CALVERTII,
	apt_app_SSALR,
	apt_app_SSALF,
	apt_app_SALS,
	apt_app_MALSR,
	apt_app_MALSF,
	apt_app_MALS,
	apt_app_ODALS,
	apt_app_RAIL,
	// Approach lights (810)
	apt_app_none_810 = 1,
	apt_app_SSALS_810,
	apt_app_SALSF_810,
	apt_app_ALSFI_810,
	apt_app_ALSFII_810,
	apt_app_ODALS_810,
	apt_app_CALVERTI_810,
	apt_app_CALVERTII_810,

	// Shoulder codes
	apt_shoulder_none = 0,
	apt_shoulder_asphalt,
	apt_shoulder_concrete,

	// Runway markings
	apt_mark_none = 0,
	apt_mark_visual,
	apt_mark_non_precision,
	apt_mark_precision,
	apt_mark_non_precision_UK,	// 850 only
	apt_mark_precision_UK,
	// Helipad Markings
	apt_mark_heli_default = 0,	// 850 only

	// Airport beacons
	apt_beacon_none = 0,
	apt_beacon_airport,
	apt_beacon_seaport,
	apt_beacon_heliport,
	apt_beacon_military,

	// Sign codes
	apt_sign_small = 1,
	apt_sign_medium,
	apt_sign_large,
	apt_sign_large_distance,
	apt_sign_small_distance,

	// Sign Style
	apt_sign_style_default = 0,

	// Linear feature codes
	apt_line_none = 0,
	apt_line_solid_yellow,
	apt_line_broken_yellow,
	apt_line_double_solid_yellow,
	apt_line_runway_hold,
	apt_line_other_hold,
	apt_line_ils_hold,
	apt_line_ils_center,
	apt_line_wide_broken_yellow,
	apt_line_wide_double_broken_yellow,
	apt_line_solid_white = 20,
	apt_line_chequered_white,
	apt_line_broken_white,
	apt_line_Bsolid_yellow = 51,
	apt_line_Bbroken_yellow,
	apt_line_Bdouble_solid_yellow,
	apt_line_Brunway_hold,
	apt_line_Bother_hold,
	apt_line_Bils_hold,
	apt_line_Bils_center,
	apt_line_Bwide_broken_yellow,
	apt_line_Bwide_double_broken_yellow,
	apt_light_taxi_centerline = 101,
	apt_light_taxi_edge,
	apt_light_hold_short,
	apt_light_hold_short_flash,
	apt_light_hold_short_centerline,
	apt_light_bounary,
	
	// ATC Crap
	
	apt_pattern_left = 1,
	apt_pattern_right = 2,
	
	atc_traffic_heavies = 1,
	atc_traffic_jets = 2,
	atc_traffic_turbos = 4,
	atc_traffic_props = 8,
	atc_traffic_helis = 16,
	atc_traffic_fighters = 32,
	
	atc_traffic_all = (atc_traffic_heavies|atc_traffic_jets|atc_traffic_turbos|atc_traffic_props|atc_traffic_helis|atc_traffic_fighters),
	
	atc_op_arrivals = 1,
	atc_op_departures = 2,
	atc_op_all = (atc_op_arrivals | atc_op_departures),
	
	atc_ramp_misc = 0,
	atc_ramp_gate = 1,
	atc_ramp_tie_down = 2,
	atc_ramp_hangar = 3,
	
	atc_width_A = 0,
	atc_width_B = 1,
	atc_width_C = 2,
	atc_width_D = 3,
	atc_width_E = 4,
	atc_width_F = 5,

	ramp_operation_none = 0,
	ramp_operation_general_aviation = 1,
	ramp_operation_airline = 2,
	ramp_operation_cargo = 3,
	ramp_operation_military = 4,
	
	//First entry of the service truck types
	apt_truck_baggage_loader = 0,
	apt_truck_baggage_train,
	apt_truck_crew_car,
	apt_truck_crew_ferrari,
	apt_truck_crew_limo,
	apt_truck_fuel_jet,
	apt_truck_fuel_liner,
	apt_truck_fuel_prop,
	apt_truck_food,
	apt_truck_gpu,
	apt_truck_pushback,

	apt_truck_destination_fuel_farm = 0,
	apt_truck_destination_baggage_hall
	
};


#endif
