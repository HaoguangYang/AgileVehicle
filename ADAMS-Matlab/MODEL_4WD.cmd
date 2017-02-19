!
!-------------------------- Default Units for Model ---------------------------!
!
!
defaults units  &
   length = meter  &
   angle = deg  &
   force = newton  &
   mass = kg  &
   time = sec
!
defaults units  &
   coordinate_system_type = cartesian  &
   orientation_type = body123
!
!------------------------ Default Attributes for Model ------------------------!
!
!
defaults attributes  &
   inheritance = bottom_up  &
   icon_visibility = off  &
   grid_visibility = off  &
   size_of_icons = 5.0E-002  &
   spacing_for_grid = 1.0
!
!------------------------------ Adams/View Model ------------------------------!
!
!
model create  &
   model_name = MODEL_1
!
view erase
!
!-------------------------------- Data storage --------------------------------!
!
!
data_element create variable  &
   variable_name = .MODEL_1.SteeringAngle_fl  &
   adams_id = 185  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.SteeringAngle_fr  &
   adams_id = 186  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.SteeringAngle_rl  &
   adams_id = 187  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.SteeringAngle_rr  &
   adams_id = 188  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.DrvTrq_fl  &
   adams_id = 189  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.DrvTrq_fr  &
   adams_id = 190  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.DrvTrq_rl  &
   adams_id = 191  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.DrvTrq_rr  &
   adams_id = 192  &
   initial_condition = 0.0  &
   function = ""
!
!--------------------------------- Materials ----------------------------------!
!
!
material create  &
   material_name = .MODEL_1.steel  &
   adams_id = 6  &
   youngs_modulus = 2.07E+011  &
   poissons_ratio = 0.29  &
   density = 7801.0
!
material create  &
   material_name = .MODEL_1.aluminum  &
   adams_id = 4  &
   youngs_modulus = 7.1705E+010  &
   poissons_ratio = 0.33  &
   density = 2740.0
!
material create  &
   material_name = .MODEL_1.wood  &
   adams_id = 3  &
   youngs_modulus = 1.1E+010  &
   poissons_ratio = 0.33  &
   density = 438.0  &
   comments = "Douglas Fir"
!
material create  &
   material_name = .MODEL_1.stainless  &
   adams_id = 5  &
   youngs_modulus = 1.9E+011  &
   poissons_ratio = 0.305  &
   density = 7750.0  &
   comments = "Stainless Steel (18-8)"
!
!-------------------------------- Rigid Parts ---------------------------------!
!
! Create parts and their dependent markers and graphics
!
!----------------------------------- ground -----------------------------------!
!
!
! ****** Ground Part ******
!
defaults model  &
   part_name = ground
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.ground.Road_ref_1  &
   adams_id = 30  &
   location = 0.7, 0.0, -0.55  &
   orientation = -90.0d, 0.0d, 90.0d
!
marker attributes  &
   marker_name = .MODEL_1.ground.Road_ref_1  &
   visibility = off
!
! ****** Floating Markers for current part ******
!
floating_marker create  &
   floating_marker_name = .MODEL_1.ground.Tires_tire_jf_1  &
   adams_id = 31
!
floating_marker create  &
   floating_marker_name = .MODEL_1.ground.Tire_rr_tire_jf_1  &
   adams_id = 295
!
floating_marker create  &
   floating_marker_name = .MODEL_1.ground.Tire_rl_tire_jf_1  &
   adams_id = 33
!
floating_marker create  &
   floating_marker_name = .MODEL_1.ground.Tire_fr_tire_jf_1  &
   adams_id = 296
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.ground  &
   material_type = .MODEL_1.steel
!
part attributes  &
   part_name = .MODEL_1.ground  &
   name_visibility = off
!
!----------------------------------- Chasis -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Chasis  &
   adams_id = 2  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Chasis
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_1  &
   adams_id = 1  &
   location = -0.5, 1.8, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.cm  &
   adams_id = 35  &
   location = 7.71296767E-004, 1.0654302433, -3.54715387E-004  &
   orientation = -2.3379466053E-002d, -5.8229643536E-002d, 0.1625187748d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_10  &
   adams_id = 10  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_157  &
   adams_id = 157  &
   location = -0.75, 0.8, 1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_211  &
   adams_id = 211  &
   location = 0.75, 0.8, 1.0  &
   orientation = 90.0d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_261  &
   adams_id = 261  &
   location = 0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_280  &
   adams_id = 280  &
   location = -0.75, 0.8, -1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_281  &
   adams_id = 281  &
   location = -0.75, 0.8, 1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_282  &
   adams_id = 282  &
   location = 0.75, 0.8, 1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_283  &
   adams_id = 283  &
   location = 0.8, 0.8, -1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Chasis.MARKER_284  &
   adams_id = 284  &
   location = -0.45, 0.8, 1.25  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Chasis  &
   density = 100.0
!
! ****** Graphics for current part ******
!
geometry create shape block  &
   block_name = .MODEL_1.Chasis.BOX_1  &
   adams_id = 1  &
   corner_marker = .MODEL_1.Chasis.MARKER_1  &
   diag_corner_coords = 1.0, -1.0, 2.0
!
geometry create shape plate  &
   plate_name = .MODEL_1.Chasis.PLATE_167  &
   marker_name = .MODEL_1.Chasis.MARKER_280, .MODEL_1.Chasis.MARKER_281,  &
                 .MODEL_1.Chasis.MARKER_282, .MODEL_1.Chasis.MARKER_283  &
   width = 5.0E-002  &
   radius = 0.1
!
geometry create shape block  &
   block_name = .MODEL_1.Chasis.BOX_167  &
   adams_id = 167  &
   corner_marker = .MODEL_1.Chasis.MARKER_284  &
   diag_corner_coords = 0.9, -0.4, -2.5
!
part attributes  &
   part_name = .MODEL_1.Chasis  &
   color = SILVER  &
   visibility = on  &
   name_visibility = off
!
!---------------------------------- Motor_fl ----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Motor_fl  &
   adams_id = 37  &
   location = 0.0, 0.0, 0.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Motor_fl
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_62  &
   adams_id = 162  &
   location = -0.71, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.cm  &
   adams_id = 163  &
   location = -0.6144451278, 0.3032289672, -1.0  &
   orientation = -89.9999999886d, 80.8484037893d, 179.999999995d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_74  &
   adams_id = 164  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_75  &
   adams_id = 165  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_82  &
   adams_id = 166  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_83  &
   adams_id = 167  &
   location = -0.53, 0.3, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_84  &
   adams_id = 168  &
   location = -0.619257043, 0.3006733749, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_98  &
   adams_id = 169  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_100  &
   adams_id = 170  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_102  &
   adams_id = 171  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 180.0d, 0.0d, 22.5620710452d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_289  &
   adams_id = 289  &
   location = -0.71, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_318  &
   adams_id = 318  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_320  &
   adams_id = 320  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_322  &
   adams_id = 322  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_356  &
   adams_id = 356  &
   location = -0.75, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Motor_fl  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.Motor_fl.CYLINDER_47  &
   adams_id = 91  &
   center_marker = .MODEL_1.Motor_fl.MARKER_62  &
   angle_extent = 360.0  &
   length = 0.18  &
   radius = 7.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
geometry create shape link  &
   link_name = .MODEL_1.Motor_fl.LINK_58  &
   i_marker = .MODEL_1.Motor_fl.MARKER_74  &
   j_marker = .MODEL_1.Motor_fl.MARKER_75  &
   width = 3.0E-002  &
   depth = 2.0E-002
!
geometry create shape plate  &
   plate_name = .MODEL_1.Motor_fl.PLATE_58  &
   marker_name = .MODEL_1.Motor_fl.MARKER_82, .MODEL_1.Motor_fl.MARKER_83,  &
                 .MODEL_1.Motor_fl.MARKER_84  &
   width = 2.0E-002  &
   radius = 2.0E-002
!
part attributes  &
   part_name = .MODEL_1.Motor_fl  &
   color = RED  &
   name_visibility = off
!
!---------------------------------- Motor_rr ----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Motor_rr  &
   adams_id = 17  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Motor_rr
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_62  &
   adams_id = 62  &
   location = -0.71, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.cm  &
   adams_id = 297  &
   location = -0.6144451278, 0.3032289672, -1.0  &
   orientation = -89.9999999886d, 80.8484037893d, 179.999999995d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_74  &
   adams_id = 74  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_75  &
   adams_id = 75  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_82  &
   adams_id = 82  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_83  &
   adams_id = 83  &
   location = -0.53, 0.3, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_84  &
   adams_id = 84  &
   location = -0.619257043, 0.3006733749, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_98  &
   adams_id = 98  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_100  &
   adams_id = 100  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_102  &
   adams_id = 102  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 180.0d, 0.0d, 22.5620710452d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_285  &
   adams_id = 285  &
   location = -0.71, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_330  &
   adams_id = 330  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_332  &
   adams_id = 332  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_334  &
   adams_id = 334  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rr.MARKER_362  &
   adams_id = 362  &
   location = -0.75, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Motor_rr  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.Motor_rr.CYLINDER_47  &
   adams_id = 47  &
   center_marker = .MODEL_1.Motor_rr.MARKER_62  &
   angle_extent = 360.0  &
   length = 0.18  &
   radius = 7.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
geometry create shape link  &
   link_name = .MODEL_1.Motor_rr.LINK_58  &
   i_marker = .MODEL_1.Motor_rr.MARKER_74  &
   j_marker = .MODEL_1.Motor_rr.MARKER_75  &
   width = 3.0E-002  &
   depth = 2.0E-002
!
geometry create shape plate  &
   plate_name = .MODEL_1.Motor_rr.PLATE_58  &
   marker_name = .MODEL_1.Motor_rr.MARKER_82, .MODEL_1.Motor_rr.MARKER_83,  &
                 .MODEL_1.Motor_rr.MARKER_84  &
   width = 2.0E-002  &
   radius = 2.0E-002
!
part attributes  &
   part_name = .MODEL_1.Motor_rr  &
   color = RED  &
   name_visibility = off
!
!---------------------------------- Susp_fl -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Susp_fl  &
   adams_id = 38  &
   location = 0.0, 0.0, 0.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Susp_fl
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_9  &
   adams_id = 172  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.cm  &
   adams_id = 173  &
   location = -0.7030892184, 0.7251151107, -1.0  &
   orientation = 90.0d, 73.0915190907d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_11  &
   adams_id = 174  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_79  &
   adams_id = 175  &
   location = -0.75, 0.7, -1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_80  &
   adams_id = 176  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_81  &
   adams_id = 177  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_91  &
   adams_id = 178  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_93  &
   adams_id = 179  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_95  &
   adams_id = 180  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 90.0d, 90.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Susp_fl  &
   material_type = .MODEL_1.aluminum
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.Susp_fl.CYLINDER_9  &
   adams_id = 94  &
   center_marker = .MODEL_1.Susp_fl.MARKER_9  &
   angle_extent = 360.0  &
   length = 0.1  &
   radius = 5.0E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
geometry create shape plate  &
   plate_name = .MODEL_1.Susp_fl.PLATE_59  &
   marker_name = .MODEL_1.Susp_fl.MARKER_79, .MODEL_1.Susp_fl.MARKER_80,  &
                 .MODEL_1.Susp_fl.MARKER_81  &
   width = 2.0E-002  &
   radius = 5.0E-002
!
part attributes  &
   part_name = .MODEL_1.Susp_fl  &
   color = CYAN  &
   visibility = on  &
   name_visibility = off
!
!---------------------------- spring_damper_flSL1 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_flSL1  &
   adams_id = 39  &
   location = 0.0, 0.0, 0.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_flSL1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL1.flSL_hinge  &
   adams_id = 181  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL1.cm  &
   adams_id = 182  &
   location = -0.60345, 0.4141169314, -0.9002602382  &
   orientation = 82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL1.MARKER_97  &
   adams_id = 183  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL1.MARKER_104  &
   adams_id = 184  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL1.MARKER_160  &
   adams_id = 185  &
   location = -0.60345, 0.5132338627, -0.9135204763  &
   orientation = -97.62d, 0.0d, -60.9293412018d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL1.MARKER_263  &
   adams_id = 263  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL1.MARKER_317  &
   adams_id = 317  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_flSL1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flSL1.CYLINDER_56  &
   adams_id = 96  &
   center_marker = .MODEL_1.spring_damper_flSL1.flSL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_flSL1  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_flSU1 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_flSU1  &
   adams_id = 40  &
   location = 0.0, 0.0, 0.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_flSU1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU1.flSU_Hinge  &
   adams_id = 186  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU1.cm  &
   adams_id = 187  &
   location = -0.60345, 0.6008830686, -0.9252397618  &
   orientation = -97.62d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU1.MARKER_92  &
   adams_id = 188  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU1.MARKER_103  &
   adams_id = 189  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU1.MARKER_161  &
   adams_id = 190  &
   location = -0.60345, 0.5132338627, -0.9135204763  &
   orientation = -97.62d, 0.0d, -60.9293412018d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU1.MARKER_262  &
   adams_id = 262  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_flSU1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flSU1.CYLINDER_53  &
   adams_id = 97  &
   center_marker = .MODEL_1.spring_damper_flSU1.flSU_Hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_flSU1  &
   color = GREEN  &
   visibility = on  &
   name_visibility = off
!
!---------------------------------- Susp_rr -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Susp_rr  &
   adams_id = 10  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Susp_rr
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.MARKER_9  &
   adams_id = 9  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.cm  &
   adams_id = 43  &
   location = -0.7030892184, 0.7251151107, -1.0  &
   orientation = 90.0d, 73.0915190907d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.MARKER_11  &
   adams_id = 11  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.MARKER_79  &
   adams_id = 79  &
   location = -0.75, 0.7, -1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.MARKER_80  &
   adams_id = 80  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.MARKER_81  &
   adams_id = 81  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.MARKER_91  &
   adams_id = 91  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.MARKER_93  &
   adams_id = 93  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rr.MARKER_95  &
   adams_id = 95  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 90.0d, 90.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Susp_rr  &
   material_type = .MODEL_1.aluminum
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.Susp_rr.CYLINDER_9  &
   adams_id = 9  &
   center_marker = .MODEL_1.Susp_rr.MARKER_9  &
   angle_extent = 360.0  &
   length = 0.1  &
   radius = 5.0E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
geometry create shape plate  &
   plate_name = .MODEL_1.Susp_rr.PLATE_59  &
   marker_name = .MODEL_1.Susp_rr.MARKER_79, .MODEL_1.Susp_rr.MARKER_80,  &
                 .MODEL_1.Susp_rr.MARKER_81  &
   width = 2.0E-002  &
   radius = 5.0E-002
!
part attributes  &
   part_name = .MODEL_1.Susp_rr  &
   color = CYAN  &
   name_visibility = off
!
!---------------------------- spring_damper_flSL2 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_flSL2  &
   adams_id = 41  &
   location = 0.0, 0.0, 0.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_flSL2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL2.flSL_hinge  &
   adams_id = 191  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL2.cm  &
   adams_id = 192  &
   location = -0.60345, 0.4141169314, -1.0997397618  &
   orientation = 97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL2.MARKER_90  &
   adams_id = 193  &
   location = -0.60345, 0.5132338627, -1.0864795237  &
   orientation = -82.38d, 0.0d, 68.638804689d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL2.MARKER_99  &
   adams_id = 194  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL2.MARKER_108  &
   adams_id = 195  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL2.MARKER_267  &
   adams_id = 267  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSL2.MARKER_319  &
   adams_id = 319  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_flSL2  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flSL2.CYLINDER_50  &
   adams_id = 98  &
   center_marker = .MODEL_1.spring_damper_flSL2.flSL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_flSL2  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_flSU2 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_flSU2  &
   adams_id = 42  &
   location = 0.0, 0.0, 0.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_flSU2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU2.flSU_hinge  &
   adams_id = 196  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU2.cm  &
   adams_id = 197  &
   location = -0.60345, 0.6008830686, -1.0747602382  &
   orientation = -82.38d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU2.MARKER_89  &
   adams_id = 198  &
   location = -0.60345, 0.5132338627, -1.0864795237  &
   orientation = -82.38d, 0.0d, 68.638804689d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU2.MARKER_96  &
   adams_id = 199  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU2.MARKER_107  &
   adams_id = 200  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flSU2.MARKER_266  &
   adams_id = 266  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_flSU2  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flSU2.CYLINDER_54  &
   adams_id = 99  &
   center_marker = .MODEL_1.spring_damper_flSU2.flSU_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.55E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_flSU2  &
   color = GREEN  &
   visibility = on  &
   name_visibility = off
!
!---------------------------- spring_damper_rrSU1 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rrSU1  &
   adams_id = 24  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rrSU1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU1.MARKER_159  &
   adams_id = 159  &
   location = -0.60345, 0.5132338627, -0.9135204763  &
   orientation = -97.62d, 0.0d, 119.0706587982d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU1.rrSU_Hinge  &
   adams_id = 69  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU1.cm  &
   adams_id = 298  &
   location = -0.60345, 0.6008830686, -0.9252397618  &
   orientation = -97.62d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU1.MARKER_92  &
   adams_id = 92  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU1.MARKER_103  &
   adams_id = 103  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rrSU1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrSU1.CYLINDER_53  &
   adams_id = 53  &
   center_marker = .MODEL_1.spring_damper_rrSU1.rrSU_Hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rrSU1  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_rrSL1 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rrSL1  &
   adams_id = 27  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rrSL1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL1.MARKER_158  &
   adams_id = 158  &
   location = -0.60345, 0.5132338627, -0.9135204763  &
   orientation = -97.62d, 0.0d, 119.0706587982d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL1.rrSL_hinge  &
   adams_id = 72  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL1.cm  &
   adams_id = 301  &
   location = -0.60345, 0.4141169314, -0.9002602382  &
   orientation = 82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL1.MARKER_97  &
   adams_id = 97  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL1.MARKER_104  &
   adams_id = 104  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL1.MARKER_331  &
   adams_id = 331  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rrSL1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrSL1.CYLINDER_56  &
   adams_id = 56  &
   center_marker = .MODEL_1.spring_damper_rrSL1.rrSL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rrSL1  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_rrSU2 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rrSU2  &
   adams_id = 25  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rrSU2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU2.rrSU_hinge  &
   adams_id = 70  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU2.cm  &
   adams_id = 299  &
   location = -0.60345, 0.6008830686, -1.0747602382  &
   orientation = -82.38d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU2.MARKER_89  &
   adams_id = 89  &
   location = -0.60345, 0.5132338627, -1.0864795237  &
   orientation = -82.38d, 0.0d, 68.638804689d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU2.MARKER_96  &
   adams_id = 96  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSU2.MARKER_107  &
   adams_id = 107  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rrSU2  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrSU2.CYLINDER_54  &
   adams_id = 54  &
   center_marker = .MODEL_1.spring_damper_rrSU2.rrSU_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.55E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rrSU2  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_rrSL2 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rrSL2  &
   adams_id = 23  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rrSL2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL2.rrSL_hinge  &
   adams_id = 67  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL2.cm  &
   adams_id = 68  &
   location = -0.60345, 0.4141169314, -1.0997397618  &
   orientation = 97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL2.MARKER_90  &
   adams_id = 90  &
   location = -0.60345, 0.5132338627, -1.0864795237  &
   orientation = -82.38d, 0.0d, 68.638804689d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL2.MARKER_99  &
   adams_id = 99  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL2.MARKER_108  &
   adams_id = 108  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrSL2.MARKER_329  &
   adams_id = 329  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rrSL2  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrSL2.CYLINDER_50  &
   adams_id = 52  &
   center_marker = .MODEL_1.spring_damper_rrSL2.rrSL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rrSL2  &
   color = GREEN  &
   name_visibility = off
!
!----------------------------- spring_damper_rrCU -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rrCU  &
   adams_id = 26  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rrCU
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCU.rrCU_hinge  &
   adams_id = 71  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 90.0d, 18.49d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCU.cm  &
   adams_id = 300  &
   location = -0.5717360862, 0.6051620979, -1.0  &
   orientation = -90.0d, -18.49d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCU.MARKER_86  &
   adams_id = 86  &
   location = -0.5668778276, 0.5906758042, -1.0  &
   orientation = -90.0d, -18.49d, 165.6236880607d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCU.MARKER_94  &
   adams_id = 94  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCU.MARKER_105  &
   adams_id = 105  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rrCU  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrCU.CYLINDER_55  &
   adams_id = 55  &
   center_marker = .MODEL_1.spring_damper_rrCU.rrCU_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rrCU  &
   color = MAIZE  &
   name_visibility = off
!
!----------------------------- spring_damper_rrCL -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rrCL  &
   adams_id = 28  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rrCL
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCL.rrCL_hinge  &
   adams_id = 73  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -90.0d, -18.49d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCL.cm  &
   adams_id = 302  &
   location = -0.5351639138, 0.4958379021, -1.0  &
   orientation = -90.0d, -18.49d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCL.MARKER_85  &
   adams_id = 85  &
   location = -0.5668778276, 0.5906758042, -1.0  &
   orientation = -90.0d, -18.49d, 165.6236880607d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCL.MARKER_101  &
   adams_id = 101  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 180.0d, 0.0d, 22.5620710452d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCL.MARKER_106  &
   adams_id = 106  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rrCL.MARKER_333  &
   adams_id = 333  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rrCL  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrCL.CYLINDER_57  &
   adams_id = 57  &
   center_marker = .MODEL_1.spring_damper_rrCL.rrCL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rrCL  &
   color = MAIZE  &
   name_visibility = off
!
!---------------------------------- Motor_fr ----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Motor_fr  &
   adams_id = 29  &
   location = 0.0, 0.0, 2.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Motor_fr
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_62  &
   adams_id = 109  &
   location = -0.71, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.cm  &
   adams_id = 110  &
   location = -0.6144451278, 0.3032289672, -1.0  &
   orientation = -89.9999999886d, 80.8484037893d, 179.999999995d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_74  &
   adams_id = 111  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_75  &
   adams_id = 112  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_82  &
   adams_id = 113  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_83  &
   adams_id = 114  &
   location = -0.53, 0.3, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_84  &
   adams_id = 115  &
   location = -0.619257043, 0.3006733749, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_98  &
   adams_id = 116  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_100  &
   adams_id = 117  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_102  &
   adams_id = 118  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 180.0d, 0.0d, 22.5620710452d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_287  &
   adams_id = 287  &
   location = -0.71, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_324  &
   adams_id = 324  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_326  &
   adams_id = 326  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_328  &
   adams_id = 328  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fr.MARKER_358  &
   adams_id = 358  &
   location = -0.75, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Motor_fr  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.Motor_fr.CYLINDER_47  &
   adams_id = 73  &
   center_marker = .MODEL_1.Motor_fr.MARKER_62  &
   angle_extent = 360.0  &
   length = 0.18  &
   radius = 7.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
geometry create shape link  &
   link_name = .MODEL_1.Motor_fr.LINK_58  &
   i_marker = .MODEL_1.Motor_fr.MARKER_74  &
   j_marker = .MODEL_1.Motor_fr.MARKER_75  &
   width = 3.0E-002  &
   depth = 2.0E-002
!
geometry create shape plate  &
   plate_name = .MODEL_1.Motor_fr.PLATE_58  &
   marker_name = .MODEL_1.Motor_fr.MARKER_82, .MODEL_1.Motor_fr.MARKER_83,  &
                 .MODEL_1.Motor_fr.MARKER_84  &
   width = 2.0E-002  &
   radius = 2.0E-002
!
part attributes  &
   part_name = .MODEL_1.Motor_fr  &
   color = RED  &
   name_visibility = off
!
!---------------------------------- Susp_fr -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Susp_fr  &
   adams_id = 30  &
   location = 0.0, 0.0, 2.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Susp_fr
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.MARKER_9  &
   adams_id = 119  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.cm  &
   adams_id = 120  &
   location = -0.7030892184, 0.7251151107, -1.0  &
   orientation = 90.0d, 73.0915190907d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.MARKER_11  &
   adams_id = 121  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.MARKER_79  &
   adams_id = 122  &
   location = -0.75, 0.7, -1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.MARKER_80  &
   adams_id = 123  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.MARKER_81  &
   adams_id = 124  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.MARKER_91  &
   adams_id = 125  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.MARKER_93  &
   adams_id = 126  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_fr.MARKER_95  &
   adams_id = 127  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 90.0d, 90.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Susp_fr  &
   material_type = .MODEL_1.aluminum
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.Susp_fr.CYLINDER_9  &
   adams_id = 82  &
   center_marker = .MODEL_1.Susp_fr.MARKER_9  &
   angle_extent = 360.0  &
   length = 0.1  &
   radius = 5.0E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
geometry create shape plate  &
   plate_name = .MODEL_1.Susp_fr.PLATE_59  &
   marker_name = .MODEL_1.Susp_fr.MARKER_79, .MODEL_1.Susp_fr.MARKER_80,  &
                 .MODEL_1.Susp_fr.MARKER_81  &
   width = 2.0E-002  &
   radius = 5.0E-002
!
part attributes  &
   part_name = .MODEL_1.Susp_fr  &
   color = CYAN  &
   visibility = on  &
   name_visibility = off
!
!---------------------------- spring_damper_frSL1 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_frSL1  &
   adams_id = 31  &
   location = 0.0, 0.0, 2.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_frSL1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL1.MARKER_160  &
   adams_id = 160  &
   location = -0.60345, 0.5132338627, -0.9135204763  &
   orientation = -97.62d, 0.0d, -60.9293412018d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL1.frSL_hinge  &
   adams_id = 128  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL1.cm  &
   adams_id = 129  &
   location = -0.60345, 0.4141169314, -0.9002602382  &
   orientation = 82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL1.MARKER_97  &
   adams_id = 131  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL1.MARKER_104  &
   adams_id = 132  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL1.MARKER_269  &
   adams_id = 269  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL1.MARKER_325  &
   adams_id = 325  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_frSL1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frSL1.CYLINDER_56  &
   adams_id = 85  &
   center_marker = .MODEL_1.spring_damper_frSL1.frSL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_frSL1  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_frSU1 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_frSU1  &
   adams_id = 32  &
   location = 0.0, 0.0, 2.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_frSU1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU1.MARKER_161  &
   adams_id = 161  &
   location = -0.60345, 0.5132338627, -0.9135204763  &
   orientation = -97.62d, 0.0d, -60.9293412018d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU1.frSU_Hinge  &
   adams_id = 133  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU1.cm  &
   adams_id = 134  &
   location = -0.60345, 0.6008830686, -0.9252397618  &
   orientation = -97.62d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU1.MARKER_92  &
   adams_id = 135  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU1.MARKER_103  &
   adams_id = 136  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU1.MARKER_268  &
   adams_id = 268  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_frSU1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frSU1.CYLINDER_53  &
   adams_id = 86  &
   center_marker = .MODEL_1.spring_damper_frSU1.frSU_Hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_frSU1  &
   color = GREEN  &
   visibility = on  &
   name_visibility = off
!
!---------------------------- spring_damper_frSL2 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_frSL2  &
   adams_id = 33  &
   location = 0.0, 0.0, 2.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_frSL2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL2.frSL_hinge  &
   adams_id = 137  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL2.cm  &
   adams_id = 138  &
   location = -0.60345, 0.4141169314, -1.0997397618  &
   orientation = 97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL2.MARKER_90  &
   adams_id = 139  &
   location = -0.60345, 0.5132338627, -1.0864795237  &
   orientation = -82.38d, 0.0d, 68.638804689d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL2.MARKER_99  &
   adams_id = 140  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL2.MARKER_108  &
   adams_id = 141  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL2.MARKER_273  &
   adams_id = 273  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSL2.MARKER_323  &
   adams_id = 323  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_frSL2  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frSL2.CYLINDER_50  &
   adams_id = 87  &
   center_marker = .MODEL_1.spring_damper_frSL2.frSL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_frSL2  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_frSU2 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_frSU2  &
   adams_id = 34  &
   location = 0.0, 0.0, 2.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_frSU2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU2.frSU_hinge  &
   adams_id = 142  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU2.cm  &
   adams_id = 143  &
   location = -0.60345, 0.6008830686, -1.0747602382  &
   orientation = -82.38d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU2.MARKER_89  &
   adams_id = 144  &
   location = -0.60345, 0.5132338627, -1.0864795237  &
   orientation = -82.38d, 0.0d, 68.638804689d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU2.MARKER_96  &
   adams_id = 145  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU2.MARKER_107  &
   adams_id = 146  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frSU2.MARKER_272  &
   adams_id = 272  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_frSU2  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frSU2.CYLINDER_54  &
   adams_id = 88  &
   center_marker = .MODEL_1.spring_damper_frSU2.frSU_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.55E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_frSU2  &
   color = GREEN  &
   visibility = on  &
   name_visibility = off
!
!----------------------------- spring_damper_frCU -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_frCU  &
   adams_id = 35  &
   location = 0.0, 0.0, 2.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_frCU
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCU.frCU_hinge  &
   adams_id = 147  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 90.0d, 18.49d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCU.cm  &
   adams_id = 148  &
   location = -0.5717360862, 0.6051620979, -1.0  &
   orientation = -90.0d, -18.49d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCU.MARKER_86  &
   adams_id = 149  &
   location = -0.5668778276, 0.5906758042, -1.0  &
   orientation = -90.0d, -18.49d, 165.6236880607d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCU.MARKER_94  &
   adams_id = 150  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCU.MARKER_105  &
   adams_id = 151  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCU.MARKER_270  &
   adams_id = 270  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_frCU  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frCU.CYLINDER_55  &
   adams_id = 89  &
   center_marker = .MODEL_1.spring_damper_frCU.frCU_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_frCU  &
   color = MAIZE  &
   visibility = on  &
   name_visibility = off
!
!----------------------------- spring_damper_frCL -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_frCL  &
   adams_id = 36  &
   location = 0.0, 0.0, 2.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_frCL
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCL.frCL_hinge  &
   adams_id = 152  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -90.0d, -18.49d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCL.cm  &
   adams_id = 153  &
   location = -0.5351639138, 0.4958379021, -1.0  &
   orientation = -90.0d, -18.49d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCL.MARKER_85  &
   adams_id = 154  &
   location = -0.5668778276, 0.5906758042, -1.0  &
   orientation = -90.0d, -18.49d, 165.6236880607d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCL.MARKER_101  &
   adams_id = 155  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 180.0d, 0.0d, 22.5620710452d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCL.MARKER_106  &
   adams_id = 156  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCL.MARKER_271  &
   adams_id = 271  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_frCL.MARKER_327  &
   adams_id = 327  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_frCL  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frCL.CYLINDER_57  &
   adams_id = 90  &
   center_marker = .MODEL_1.spring_damper_frCL.frCL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_frCL  &
   color = MAIZE  &
   name_visibility = off
!
!----------------------------- spring_damper_flCU -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_flCU  &
   adams_id = 43  &
   location = 0.0, 0.0, 0.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_flCU
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCU.flCU_hinge  &
   adams_id = 201  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 90.0d, 18.49d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCU.cm  &
   adams_id = 202  &
   location = -0.5717360862, 0.6051620979, -1.0  &
   orientation = -90.0d, -18.49d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCU.MARKER_86  &
   adams_id = 203  &
   location = -0.5668778276, 0.5906758042, -1.0  &
   orientation = -90.0d, -18.49d, 165.6236880607d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCU.MARKER_94  &
   adams_id = 204  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCU.MARKER_105  &
   adams_id = 205  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCU.MARKER_264  &
   adams_id = 264  &
   location = -0.60345, 0.7, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_flCU  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flCU.CYLINDER_55  &
   adams_id = 100  &
   center_marker = .MODEL_1.spring_damper_flCU.flCU_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_flCU  &
   color = MAIZE  &
   visibility = on  &
   name_visibility = off
!
!----------------------------- spring_damper_flCL -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_flCL  &
   adams_id = 44  &
   location = 0.0, 0.0, 0.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_flCL
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCL.flCL_hinge  &
   adams_id = 206  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -90.0d, -18.49d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCL.cm  &
   adams_id = 207  &
   location = -0.5351639138, 0.4958379021, -1.0  &
   orientation = -90.0d, -18.49d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCL.MARKER_85  &
   adams_id = 208  &
   location = -0.5668778276, 0.5906758042, -1.0  &
   orientation = -90.0d, -18.49d, 165.6236880607d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCL.MARKER_101  &
   adams_id = 209  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 180.0d, 0.0d, 22.5620710452d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCL.MARKER_106  &
   adams_id = 210  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCL.MARKER_265  &
   adams_id = 265  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_flCL.MARKER_321  &
   adams_id = 321  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_flCL  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flCL.CYLINDER_57  &
   adams_id = 101  &
   center_marker = .MODEL_1.spring_damper_flCL.flCL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_flCL  &
   color = MAIZE  &
   name_visibility = off
!
!---------------------------------- Motor_rl ----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Motor_rl  &
   adams_id = 45  &
   location = 0.0, 0.0, -2.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Motor_rl
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_62  &
   adams_id = 212  &
   location = -0.71, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.cm  &
   adams_id = 213  &
   location = -0.6144451278, 0.3032289672, -1.0  &
   orientation = -89.9999999886d, 80.8484037893d, 179.999999995d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_74  &
   adams_id = 214  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_75  &
   adams_id = 215  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_82  &
   adams_id = 216  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_83  &
   adams_id = 217  &
   location = -0.53, 0.3, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_84  &
   adams_id = 218  &
   location = -0.619257043, 0.3006733749, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_98  &
   adams_id = 219  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_100  &
   adams_id = 220  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_102  &
   adams_id = 221  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 180.0d, 0.0d, 22.5620710452d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_291  &
   adams_id = 291  &
   location = -0.71, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_312  &
   adams_id = 312  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_314  &
   adams_id = 314  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_316  &
   adams_id = 316  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_rl.MARKER_360  &
   adams_id = 360  &
   location = -0.75, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Motor_rl  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.Motor_rl.CYLINDER_47  &
   adams_id = 102  &
   center_marker = .MODEL_1.Motor_rl.MARKER_62  &
   angle_extent = 360.0  &
   length = 0.18  &
   radius = 7.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
geometry create shape link  &
   link_name = .MODEL_1.Motor_rl.LINK_58  &
   i_marker = .MODEL_1.Motor_rl.MARKER_74  &
   j_marker = .MODEL_1.Motor_rl.MARKER_75  &
   width = 3.0E-002  &
   depth = 2.0E-002
!
geometry create shape plate  &
   plate_name = .MODEL_1.Motor_rl.PLATE_58  &
   marker_name = .MODEL_1.Motor_rl.MARKER_82, .MODEL_1.Motor_rl.MARKER_83,  &
                 .MODEL_1.Motor_rl.MARKER_84  &
   width = 2.0E-002  &
   radius = 2.0E-002
!
part attributes  &
   part_name = .MODEL_1.Motor_rl  &
   color = RED  &
   name_visibility = off
!
!---------------------------------- Susp_rl -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.Susp_rl  &
   adams_id = 46  &
   location = 0.0, 0.0, -2.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.Susp_rl
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.MARKER_9  &
   adams_id = 222  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.cm  &
   adams_id = 223  &
   location = -0.7030892184, 0.7251151107, -1.0  &
   orientation = 90.0d, 73.0915190907d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.MARKER_11  &
   adams_id = 224  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.MARKER_79  &
   adams_id = 225  &
   location = -0.75, 0.7, -1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.MARKER_80  &
   adams_id = 226  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.MARKER_81  &
   adams_id = 227  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.MARKER_91  &
   adams_id = 228  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.MARKER_93  &
   adams_id = 229  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Susp_rl.MARKER_95  &
   adams_id = 230  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 90.0d, 90.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.Susp_rl  &
   material_type = .MODEL_1.aluminum
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.Susp_rl.CYLINDER_9  &
   adams_id = 105  &
   center_marker = .MODEL_1.Susp_rl.MARKER_9  &
   angle_extent = 360.0  &
   length = 0.1  &
   radius = 5.0E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
geometry create shape plate  &
   plate_name = .MODEL_1.Susp_rl.PLATE_59  &
   marker_name = .MODEL_1.Susp_rl.MARKER_79, .MODEL_1.Susp_rl.MARKER_80,  &
                 .MODEL_1.Susp_rl.MARKER_81  &
   width = 2.0E-002  &
   radius = 5.0E-002
!
part attributes  &
   part_name = .MODEL_1.Susp_rl  &
   color = CYAN  &
   visibility = on  &
   name_visibility = off
!
!---------------------------- spring_damper_rlSL1 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rlSL1  &
   adams_id = 47  &
   location = 0.0, 0.0, -2.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rlSL1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL1.rlSL_hinge  &
   adams_id = 231  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL1.cm  &
   adams_id = 232  &
   location = -0.60345, 0.4141169314, -0.9002602382  &
   orientation = 82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL1.MARKER_97  &
   adams_id = 233  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL1.MARKER_104  &
   adams_id = 234  &
   location = -0.60345, 0.315, -0.887  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL1.MARKER_160  &
   adams_id = 235  &
   location = -0.60345, 0.5132338627, -0.9135204763  &
   orientation = -97.62d, 0.0d, -60.9293412018d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL1.MARKER_275  &
   adams_id = 275  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL1.MARKER_311  &
   adams_id = 311  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rlSL1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlSL1.CYLINDER_56  &
   adams_id = 107  &
   center_marker = .MODEL_1.spring_damper_rlSL1.rlSL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rlSL1  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_rlSU1 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rlSU1  &
   adams_id = 48  &
   location = 0.0, 0.0, -2.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rlSU1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU1.rlSU_Hinge  &
   adams_id = 236  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU1.cm  &
   adams_id = 237  &
   location = -0.60345, 0.6008830686, -0.9252397618  &
   orientation = -97.62d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU1.MARKER_92  &
   adams_id = 238  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU1.MARKER_103  &
   adams_id = 239  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU1.MARKER_161  &
   adams_id = 240  &
   location = -0.60345, 0.5132338627, -0.9135204763  &
   orientation = -97.62d, 0.0d, -60.9293412018d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU1.MARKER_274  &
   adams_id = 274  &
   location = -0.60345, 0.7, -0.9385  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rlSU1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlSU1.CYLINDER_53  &
   adams_id = 108  &
   center_marker = .MODEL_1.spring_damper_rlSU1.rlSU_Hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rlSU1  &
   color = GREEN  &
   visibility = on  &
   name_visibility = off
!
!---------------------------- spring_damper_rlSL2 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rlSL2  &
   adams_id = 49  &
   location = 0.0, 0.0, -2.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rlSL2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL2.rlSL_hinge  &
   adams_id = 241  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -82.38d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL2.cm  &
   adams_id = 242  &
   location = -0.60345, 0.4141169314, -1.0997397618  &
   orientation = 97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL2.MARKER_90  &
   adams_id = 243  &
   location = -0.60345, 0.5132338627, -1.0864795237  &
   orientation = -82.38d, 0.0d, 68.638804689d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL2.MARKER_99  &
   adams_id = 244  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 71.8975548067d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL2.MARKER_108  &
   adams_id = 245  &
   location = -0.60345, 0.315, -1.113  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL2.MARKER_279  &
   adams_id = 279  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSL2.MARKER_313  &
   adams_id = 313  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rlSL2  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlSL2.CYLINDER_50  &
   adams_id = 109  &
   center_marker = .MODEL_1.spring_damper_rlSL2.rlSL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rlSL2  &
   color = GREEN  &
   name_visibility = off
!
!---------------------------- spring_damper_rlSU2 -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rlSU2  &
   adams_id = 50  &
   location = 0.0, 0.0, -2.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rlSU2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU2.rlSU_hinge  &
   adams_id = 246  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 97.62d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU2.cm  &
   adams_id = 247  &
   location = -0.60345, 0.6008830686, -1.0747602382  &
   orientation = -82.38d, 0.0d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU2.MARKER_89  &
   adams_id = 248  &
   location = -0.60345, 0.5132338627, -1.0864795237  &
   orientation = -82.38d, 0.0d, 68.638804689d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU2.MARKER_96  &
   adams_id = 249  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU2.MARKER_107  &
   adams_id = 250  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlSU2.MARKER_278  &
   adams_id = 278  &
   location = -0.60345, 0.7, -1.0615  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rlSU2  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlSU2.CYLINDER_54  &
   adams_id = 110  &
   center_marker = .MODEL_1.spring_damper_rlSU2.rlSU_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.55E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rlSU2  &
   color = GREEN  &
   visibility = on  &
   name_visibility = off
!
!----------------------------- spring_damper_rlCU -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rlCU  &
   adams_id = 51  &
   location = 0.0, 0.0, -2.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rlCU
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCU.rlCU_hinge  &
   adams_id = 251  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 90.0d, 18.49d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCU.cm  &
   adams_id = 252  &
   location = -0.5717360862, 0.6051620979, -1.0  &
   orientation = -90.0d, -18.49d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCU.MARKER_86  &
   adams_id = 253  &
   location = -0.5668778276, 0.5906758042, -1.0  &
   orientation = -90.0d, -18.49d, 165.6236880607d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCU.MARKER_94  &
   adams_id = 254  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCU.MARKER_105  &
   adams_id = 255  &
   location = -0.60345, 0.7, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCU.MARKER_276  &
   adams_id = 276  &
   location = -0.60345, 0.7, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rlCU  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlCU.CYLINDER_55  &
   adams_id = 111  &
   center_marker = .MODEL_1.spring_damper_rlCU.rlCU_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 2.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rlCU  &
   color = MAIZE  &
   visibility = on  &
   name_visibility = off
!
!----------------------------- spring_damper_rlCL -----------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.spring_damper_rlCL  &
   adams_id = 52  &
   location = 0.0, 0.0, -2.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.spring_damper_rlCL
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCL.rlCL_hinge  &
   adams_id = 256  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -90.0d, -18.49d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCL.cm  &
   adams_id = 257  &
   location = -0.5351639138, 0.4958379021, -1.0  &
   orientation = -90.0d, -18.49d, -90.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCL.MARKER_85  &
   adams_id = 258  &
   location = -0.5668778276, 0.5906758042, -1.0  &
   orientation = -90.0d, -18.49d, 165.6236880607d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCL.MARKER_101  &
   adams_id = 259  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 180.0d, 0.0d, 22.5620710452d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCL.MARKER_106  &
   adams_id = 260  &
   location = -0.50345, 0.401, -1.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCL.MARKER_277  &
   adams_id = 277  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.spring_damper_rlCL.MARKER_315  &
   adams_id = 315  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.spring_damper_rlCL  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlCL.CYLINDER_57  &
   adams_id = 112  &
   center_marker = .MODEL_1.spring_damper_rlCL.rlCL_hinge  &
   angle_extent = 360.0  &
   length = 0.2  &
   radius = 1.5E-002  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .MODEL_1.spring_damper_rlCL  &
   color = MAIZE  &
   name_visibility = off
!
!----------------------------------- Joints -----------------------------------!
!
!
constraint create joint revolute  &
   joint_name = .MODEL_1.steering_rr  &
   adams_id = 1  &
   i_marker_name = .MODEL_1.Chasis.MARKER_10  &
   j_marker_name = .MODEL_1.Susp_rr.MARKER_11
!
constraint attributes  &
   constraint_name = .MODEL_1.steering_rr  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_29  &
   adams_id = 29  &
   i_marker_name = .MODEL_1.spring_damper_rrSL1.MARKER_158  &
   j_marker_name = .MODEL_1.spring_damper_rrSU1.MARKER_159
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_29  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_31  &
   adams_id = 31  &
   i_marker_name = .MODEL_1.spring_damper_flSL1.MARKER_160  &
   j_marker_name = .MODEL_1.spring_damper_flSU1.MARKER_161
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_31  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_10  &
   adams_id = 10  &
   i_marker_name = .MODEL_1.spring_damper_rrCL.MARKER_85  &
   j_marker_name = .MODEL_1.spring_damper_rrCU.MARKER_86
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_10  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_30  &
   adams_id = 30  &
   i_marker_name = .MODEL_1.spring_damper_frSL1.MARKER_160  &
   j_marker_name = .MODEL_1.spring_damper_frSU1.MARKER_161
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_30  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_12  &
   adams_id = 12  &
   i_marker_name = .MODEL_1.spring_damper_rrSU2.MARKER_89  &
   j_marker_name = .MODEL_1.spring_damper_rrSL2.MARKER_90
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_12  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_13  &
   adams_id = 13  &
   i_marker_name = .MODEL_1.Susp_rr.MARKER_91  &
   j_marker_name = .MODEL_1.spring_damper_rrSU1.MARKER_92
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_13  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_14  &
   adams_id = 14  &
   i_marker_name = .MODEL_1.Susp_rr.MARKER_93  &
   j_marker_name = .MODEL_1.spring_damper_rrCU.MARKER_94
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_14  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_15  &
   adams_id = 15  &
   i_marker_name = .MODEL_1.Susp_rr.MARKER_95  &
   j_marker_name = .MODEL_1.spring_damper_rrSU2.MARKER_96
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_15  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_16  &
   adams_id = 16  &
   i_marker_name = .MODEL_1.spring_damper_rrSL1.MARKER_97  &
   j_marker_name = .MODEL_1.Motor_rr.MARKER_98
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_16  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_17  &
   adams_id = 17  &
   i_marker_name = .MODEL_1.spring_damper_rrSL2.MARKER_99  &
   j_marker_name = .MODEL_1.Motor_rr.MARKER_100
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_17  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_18  &
   adams_id = 18  &
   i_marker_name = .MODEL_1.spring_damper_rrCL.MARKER_101  &
   j_marker_name = .MODEL_1.Motor_rr.MARKER_102
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_18  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.steering_fr  &
   adams_id = 19  &
   i_marker_name = .MODEL_1.Chasis.MARKER_157  &
   j_marker_name = .MODEL_1.Susp_fr.MARKER_11
!
constraint attributes  &
   constraint_name = .MODEL_1.steering_fr  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_20  &
   adams_id = 20  &
   i_marker_name = .MODEL_1.spring_damper_frCL.MARKER_85  &
   j_marker_name = .MODEL_1.spring_damper_frCU.MARKER_86
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_20  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.steering_fl  &
   adams_id = 32  &
   i_marker_name = .MODEL_1.Chasis.MARKER_211  &
   j_marker_name = .MODEL_1.Susp_fl.MARKER_11
!
constraint attributes  &
   constraint_name = .MODEL_1.steering_fl  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_22  &
   adams_id = 22  &
   i_marker_name = .MODEL_1.spring_damper_frSU2.MARKER_89  &
   j_marker_name = .MODEL_1.spring_damper_frSL2.MARKER_90
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_22  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_23  &
   adams_id = 23  &
   i_marker_name = .MODEL_1.Susp_fr.MARKER_91  &
   j_marker_name = .MODEL_1.spring_damper_frSU1.MARKER_92
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_23  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_24  &
   adams_id = 24  &
   i_marker_name = .MODEL_1.Susp_fr.MARKER_93  &
   j_marker_name = .MODEL_1.spring_damper_frCU.MARKER_94
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_24  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_25  &
   adams_id = 25  &
   i_marker_name = .MODEL_1.Susp_fr.MARKER_95  &
   j_marker_name = .MODEL_1.spring_damper_frSU2.MARKER_96
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_25  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_26  &
   adams_id = 26  &
   i_marker_name = .MODEL_1.spring_damper_frSL1.MARKER_97  &
   j_marker_name = .MODEL_1.Motor_fr.MARKER_98
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_26  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_27  &
   adams_id = 27  &
   i_marker_name = .MODEL_1.spring_damper_frSL2.MARKER_99  &
   j_marker_name = .MODEL_1.Motor_fr.MARKER_100
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_27  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_28  &
   adams_id = 28  &
   i_marker_name = .MODEL_1.spring_damper_frCL.MARKER_101  &
   j_marker_name = .MODEL_1.Motor_fr.MARKER_102
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_28  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_33  &
   adams_id = 33  &
   i_marker_name = .MODEL_1.spring_damper_flCL.MARKER_85  &
   j_marker_name = .MODEL_1.spring_damper_flCU.MARKER_86
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_33  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_34  &
   adams_id = 34  &
   i_marker_name = .MODEL_1.spring_damper_flSU2.MARKER_89  &
   j_marker_name = .MODEL_1.spring_damper_flSL2.MARKER_90
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_34  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_35  &
   adams_id = 35  &
   i_marker_name = .MODEL_1.Susp_fl.MARKER_91  &
   j_marker_name = .MODEL_1.spring_damper_flSU1.MARKER_92
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_35  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_36  &
   adams_id = 36  &
   i_marker_name = .MODEL_1.Susp_fl.MARKER_93  &
   j_marker_name = .MODEL_1.spring_damper_flCU.MARKER_94
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_36  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_37  &
   adams_id = 37  &
   i_marker_name = .MODEL_1.Susp_fl.MARKER_95  &
   j_marker_name = .MODEL_1.spring_damper_flSU2.MARKER_96
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_37  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_38  &
   adams_id = 38  &
   i_marker_name = .MODEL_1.spring_damper_flSL1.MARKER_97  &
   j_marker_name = .MODEL_1.Motor_fl.MARKER_98
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_38  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_39  &
   adams_id = 39  &
   i_marker_name = .MODEL_1.spring_damper_flSL2.MARKER_99  &
   j_marker_name = .MODEL_1.Motor_fl.MARKER_100
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_39  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_40  &
   adams_id = 40  &
   i_marker_name = .MODEL_1.spring_damper_flCL.MARKER_101  &
   j_marker_name = .MODEL_1.Motor_fl.MARKER_102
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_40  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_41  &
   adams_id = 41  &
   i_marker_name = .MODEL_1.spring_damper_rlSL1.MARKER_160  &
   j_marker_name = .MODEL_1.spring_damper_rlSU1.MARKER_161
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_41  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.steering_rl  &
   adams_id = 42  &
   i_marker_name = .MODEL_1.Chasis.MARKER_261  &
   j_marker_name = .MODEL_1.Susp_rl.MARKER_11
!
constraint attributes  &
   constraint_name = .MODEL_1.steering_rl  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_43  &
   adams_id = 43  &
   i_marker_name = .MODEL_1.spring_damper_rlCL.MARKER_85  &
   j_marker_name = .MODEL_1.spring_damper_rlCU.MARKER_86
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_43  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_44  &
   adams_id = 44  &
   i_marker_name = .MODEL_1.spring_damper_rlSU2.MARKER_89  &
   j_marker_name = .MODEL_1.spring_damper_rlSL2.MARKER_90
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_44  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_45  &
   adams_id = 45  &
   i_marker_name = .MODEL_1.Susp_rl.MARKER_91  &
   j_marker_name = .MODEL_1.spring_damper_rlSU1.MARKER_92
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_45  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_46  &
   adams_id = 46  &
   i_marker_name = .MODEL_1.Susp_rl.MARKER_93  &
   j_marker_name = .MODEL_1.spring_damper_rlCU.MARKER_94
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_46  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_47  &
   adams_id = 47  &
   i_marker_name = .MODEL_1.Susp_rl.MARKER_95  &
   j_marker_name = .MODEL_1.spring_damper_rlSU2.MARKER_96
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_47  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_48  &
   adams_id = 48  &
   i_marker_name = .MODEL_1.spring_damper_rlSL1.MARKER_97  &
   j_marker_name = .MODEL_1.Motor_rl.MARKER_98
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_48  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_49  &
   adams_id = 49  &
   i_marker_name = .MODEL_1.spring_damper_rlSL2.MARKER_99  &
   j_marker_name = .MODEL_1.Motor_rl.MARKER_100
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_49  &
   name_visibility = off
!
constraint create joint cylindrical  &
   joint_name = .MODEL_1.JOINT_50  &
   adams_id = 50  &
   i_marker_name = .MODEL_1.spring_damper_rlCL.MARKER_101  &
   j_marker_name = .MODEL_1.Motor_rl.MARKER_102
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_50  &
   name_visibility = off
!
!----------------------------------- Forces -----------------------------------!
!
!
!---------------------------- Adams/View Variables ----------------------------!
!
!
variable create  &
   variable_name = .MODEL_1.SideSpringK  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_range = no  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 1.1E+005
!
variable create  &
   variable_name = .MODEL_1.SideSpringD  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 200.0
!
variable create  &
   variable_name = .MODEL_1.CentralSpringK  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_range = no  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 1.5E+004
!
variable create  &
   variable_name = .MODEL_1.CentralSpringD  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 2000.0
!
variable create  &
   variable_name = .MODEL_1.SideSpringL  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_range = no  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 0.41
!
variable create  &
   variable_name = .MODEL_1.CentralSpringL  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_range = no  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 0.33
!
variable create  &
   variable_name = .MODEL_1.K_R1  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 2000.0
!
variable create  &
   variable_name = .MODEL_1.K_R2  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 2.0E+004
!
!----------------------------- Simulation Scripts -----------------------------!
!
!
simulation script create  &
   sim_script_name = .MODEL_1.Last_Sim  &
   commands =   &
              "simulation single_run transient type=auto_select initial_static=yes end_time=30.0 number_of_steps=3000 model_name=.MODEL_1"
!
!-------------------------- Adams/View UDE Instances --------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
undo begin_block suppress = yes
!
ude create instance  &
   instance_name = .MODEL_1.Road  &
   definition_name = .MDI.Forces.vpg_road  &
   location = 0.7, 0.0, -0.55  &
   orientation = -90.0, 0.0, 90.0
!
ude attributes  &
   instance_name = .MODEL_1.Road  &
   color = DimGray  &
   visibility = on
!
ude create instance  &
   instance_name = .MODEL_1.Tire_fl  &
   definition_name = .MDI.Forces.vpg_tire  &
   location = 0.75, 0.3, 1.0  &
   orientation = 90.0, 90.0, 0.0
!
marker create  &
   marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_357  &
   adams_id = 357  &
   location = 0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_364  &
   adams_id = 364  &
   location = 0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_365  &
   adams_id = 365  &
   location = 0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
ude create instance  &
   instance_name = .MODEL_1.Tire_rr  &
   definition_name = .MDI.Forces.vpg_tire  &
   location = -0.75, 0.3, -1.0  &
   orientation = 90.0, 90.0, 0.0
!
marker create  &
   marker_name = .MODEL_1.Tire_rr.wheel_part.MARKER_363  &
   adams_id = 363  &
   location = -0.75, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_rr.wheel_part.MARKER_370  &
   adams_id = 370  &
   location = -0.75, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_rr.wheel_part.MARKER_371  &
   adams_id = 371  &
   location = -0.75, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
ude create instance  &
   instance_name = .MODEL_1.Tire_rl  &
   definition_name = .MDI.Forces.vpg_tire  &
   location = 0.75, 0.3, -1.0  &
   orientation = 90.0, 90.0, 0.0
!
marker create  &
   marker_name = .MODEL_1.Tire_rl.wheel_part.MARKER_361  &
   adams_id = 361  &
   location = 0.75, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_rl.wheel_part.MARKER_368  &
   adams_id = 368  &
   location = 0.75, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_rl.wheel_part.MARKER_369  &
   adams_id = 369  &
   location = 0.75, 0.3, -1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
ude create instance  &
   instance_name = .MODEL_1.Tire_fr  &
   definition_name = .MDI.Forces.vpg_tire  &
   location = -0.75, 0.3, 1.0  &
   orientation = 90.0, 90.0, 0.0
!
marker create  &
   marker_name = .MODEL_1.Tire_fr.wheel_part.MARKER_359  &
   adams_id = 359  &
   location = -0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_fr.wheel_part.MARKER_366  &
   adams_id = 366  &
   location = -0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_fr.wheel_part.MARKER_367  &
   adams_id = 367  &
   location = -0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rrS1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rrS1  &
   color = RED  &
   visibility = on
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rrC  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rrC  &
   color = RED  &
   visibility = on
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rrS2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rrS2  &
   color = RED  &
   visibility = on
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_flS1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_flS1  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_flC  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_flC  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_flS2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_flS2  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_frS1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_frS1  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_frC  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_frC  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_frS2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_frS2  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rlS1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rlS1  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rlC  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rlC  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rlS2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rlS2  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rlR1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rlR1  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rlR2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rlR2  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rlR3  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rlR3  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_flR1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_flR1  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_flR2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_flR2  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_flR3  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_flR3  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_frR1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_frR1  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_frR2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_frR2  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_frR3  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_frR3  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rrR1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rrR1  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rrR2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rrR2  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_rrR3  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_rrR3  &
   color = RED
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.Road.ref_marker  &
   object_value = .MODEL_1.ground.Road_ref_1
!
variable modify  &
   variable_name = .MODEL_1.Road.road_property_file  &
   string_value = "C:/MSC.Software/Adams_x64/2012/acar/shared_car_database.cdb/roads.tbl/road_3d_grid_example.rdf"
!
variable modify  &
   variable_name = .MODEL_1.Road.road_graphics  &
   string_value = "on"
!
ude modify instance  &
   instance_name = .MODEL_1.Road
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.cm_offset  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.center_offset  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.long_vel  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.spin_vel  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.side  &
   string_value = "left"
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.road_property_file  &
   string_value = (.MODEL_1.Road.road_property_file)
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.j_fmarker  &
   object_value = .MODEL_1.ground.Tires_tire_jf_1
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.ref_marker  &
   object_value = (.MODEL_1.Road.ref_marker.object_value)
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.wheel_tire_mass  &
   real_value = 20.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.Ixx_Iyy  &
   real_value = 0.7
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.Izz  &
   real_value = 1.2
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.property_file  &
   string_value = "C:/MSC.Software/Adams_x64/2012/acar/shared_car_database.cdb/tires.tbl/pac2002_205_55R16_tdft.tir"
!
variable modify  &
   variable_name = .MODEL_1.Tire_fl.road_name  &
   string_value = (.MODEL_1.Road)
!
ude modify instance  &
   instance_name = .MODEL_1.Tire_fl
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.cm_offset  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.center_offset  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.long_vel  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.spin_vel  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.side  &
   string_value = "right"
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.road_property_file  &
   string_value = (.MODEL_1.Road.road_property_file)
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.j_fmarker  &
   object_value = .MODEL_1.ground.Tire_rr_tire_jf_1
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.ref_marker  &
   object_value = (.MODEL_1.Road.ref_marker.object_value)
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.wheel_tire_mass  &
   real_value = 20.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.Ixx_Iyy  &
   real_value = 0.7
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.Izz  &
   real_value = 1.2
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.property_file  &
   string_value = "C:/MSC.Software/Adams_x64/2012/acar/shared_car_database.cdb/tires.tbl/pac2002_205_55R16_tdft.tir"
!
variable modify  &
   variable_name = .MODEL_1.Tire_rr.road_name  &
   string_value = (.MODEL_1.Road)
!
ude modify instance  &
   instance_name = .MODEL_1.Tire_rr
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.cm_offset  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.center_offset  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.long_vel  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.spin_vel  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.side  &
   string_value = "left"
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.road_property_file  &
   string_value = (.MODEL_1.Road.road_property_file)
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.j_fmarker  &
   object_value = .MODEL_1.ground.Tire_rl_tire_jf_1
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.ref_marker  &
   object_value = (.MODEL_1.Road.ref_marker.object_value)
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.wheel_tire_mass  &
   real_value = 20.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.Ixx_Iyy  &
   real_value = 0.7
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.Izz  &
   real_value = 1.2
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.property_file  &
   string_value = "C:/MSC.Software/Adams_x64/2012/acar/shared_car_database.cdb/tires.tbl/pac2002_205_55R16_tdft.tir"
!
variable modify  &
   variable_name = .MODEL_1.Tire_rl.road_name  &
   string_value = (.MODEL_1.Road)
!
ude modify instance  &
   instance_name = .MODEL_1.Tire_rl
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.cm_offset  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.center_offset  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.long_vel  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.spin_vel  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.side  &
   string_value = "right"
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.road_property_file  &
   string_value = (.MODEL_1.Road.road_property_file)
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.j_fmarker  &
   object_value = .MODEL_1.ground.Tire_fr_tire_jf_1
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.ref_marker  &
   object_value = (.MODEL_1.Road.ref_marker.object_value)
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.wheel_tire_mass  &
   real_value = 20.0
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.Ixx_Iyy  &
   real_value = 0.7
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.Izz  &
   real_value = 1.2
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.property_file  &
   string_value = "C:/MSC.Software/Adams_x64/2012/acar/shared_car_database.cdb/tires.tbl/pac2002_205_55R16_tdft.tir"
!
variable modify  &
   variable_name = .MODEL_1.Tire_fr.road_name  &
   string_value = (.MODEL_1.Road)
!
ude modify instance  &
   instance_name = .MODEL_1.Tire_fr
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.i_marker  &
   object_value = (.MODEL_1.spring_damper_rrSU1.MARKER_103)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.j_marker  &
   object_value = (.MODEL_1.spring_damper_rrSL1.MARKER_104)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.stiffness_coefficient  &
   real_value = (.MODEL_1.SideSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.damping_coefficient  &
   real_value = (.MODEL_1.SideSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.free_length  &
   real_value = (.MODEL_1.SideSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.i_marker  &
   object_value = (.MODEL_1.spring_damper_rrCU.MARKER_105)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.j_marker  &
   object_value = (.MODEL_1.spring_damper_rrCL.MARKER_106)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.stiffness_coefficient  &
   real_value = (.MODEL_1.CentralSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.damping_coefficient  &
   real_value = (.MODEL_1.CentralSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.free_length  &
   real_value = (.MODEL_1.CentralSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrC.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.i_marker  &
   object_value = (.MODEL_1.spring_damper_rrSU2.MARKER_107)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.j_marker  &
   object_value = (.MODEL_1.spring_damper_rrSL2.MARKER_108)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.stiffness_coefficient  &
   real_value = (.MODEL_1.SideSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.damping_coefficient  &
   real_value = (.MODEL_1.SideSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.free_length  &
   real_value = (.MODEL_1.SideSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrS2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.i_marker  &
   object_value = (.MODEL_1.spring_damper_flSU1.MARKER_262)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.j_marker  &
   object_value = (.MODEL_1.spring_damper_flSL1.MARKER_263)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.stiffness_coefficient  &
   real_value = (.MODEL_1.SideSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.damping_coefficient  &
   real_value = (.MODEL_1.SideSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.free_length  &
   real_value = (.MODEL_1.SideSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.i_marker  &
   object_value = (.MODEL_1.spring_damper_flCU.MARKER_264)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.j_marker  &
   object_value = (.MODEL_1.spring_damper_flCL.MARKER_265)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.stiffness_coefficient  &
   real_value = (.MODEL_1.CentralSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.damping_coefficient  &
   real_value = (.MODEL_1.CentralSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.free_length  &
   real_value = (.MODEL_1.CentralSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flC.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.i_marker  &
   object_value = (.MODEL_1.spring_damper_flSU2.MARKER_266)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.j_marker  &
   object_value = (.MODEL_1.spring_damper_flSL2.MARKER_267)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.stiffness_coefficient  &
   real_value = (.MODEL_1.SideSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.damping_coefficient  &
   real_value = (.MODEL_1.SideSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.free_length  &
   real_value = (.MODEL_1.SideSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flS2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.i_marker  &
   object_value = (.MODEL_1.spring_damper_frSU1.MARKER_268)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.j_marker  &
   object_value = (.MODEL_1.spring_damper_frSL1.MARKER_269)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.stiffness_coefficient  &
   real_value = (.MODEL_1.SideSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.damping_coefficient  &
   real_value = (.MODEL_1.SideSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.free_length  &
   real_value = (.MODEL_1.SideSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.i_marker  &
   object_value = (.MODEL_1.spring_damper_frCU.MARKER_270)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.j_marker  &
   object_value = (.MODEL_1.spring_damper_frCL.MARKER_271)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.stiffness_coefficient  &
   real_value = (.MODEL_1.CentralSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.damping_coefficient  &
   real_value = (.MODEL_1.CentralSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.free_length  &
   real_value = (.MODEL_1.CentralSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frC.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.i_marker  &
   object_value = (.MODEL_1.spring_damper_frSU2.MARKER_272)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.j_marker  &
   object_value = (.MODEL_1.spring_damper_frSL2.MARKER_273)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.stiffness_coefficient  &
   real_value = (.MODEL_1.SideSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.damping_coefficient  &
   real_value = (.MODEL_1.SideSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.free_length  &
   real_value = (.MODEL_1.SideSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frS2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.i_marker  &
   object_value = (.MODEL_1.spring_damper_rlSU1.MARKER_274)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.j_marker  &
   object_value = (.MODEL_1.spring_damper_rlSL1.MARKER_275)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.stiffness_coefficient  &
   real_value = (.MODEL_1.SideSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.damping_coefficient  &
   real_value = (.MODEL_1.SideSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.free_length  &
   real_value = (.MODEL_1.SideSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.i_marker  &
   object_value = (.MODEL_1.spring_damper_rlCU.MARKER_276)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.j_marker  &
   object_value = (.MODEL_1.spring_damper_rlCL.MARKER_277)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.stiffness_coefficient  &
   real_value = (.MODEL_1.CentralSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.damping_coefficient  &
   real_value = (.MODEL_1.CentralSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.free_length  &
   real_value = (.MODEL_1.CentralSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlC.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.i_marker  &
   object_value = (.MODEL_1.spring_damper_rlSU2.MARKER_278)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.j_marker  &
   object_value = (.MODEL_1.spring_damper_rlSL2.MARKER_279)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.stiffness_coefficient  &
   real_value = (.MODEL_1.SideSpringK)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.damping_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.damping_coefficient  &
   real_value = (.MODEL_1.SideSpringD)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.free_length  &
   real_value = (.MODEL_1.SideSpringL)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlS2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.i_marker  &
   object_value = (.MODEL_1.spring_damper_rlSL1.MARKER_311)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.j_marker  &
   object_value = (.MODEL_1.Motor_rl.MARKER_312)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlR1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.i_marker  &
   object_value = (.MODEL_1.spring_damper_rlSL2.MARKER_313)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.j_marker  &
   object_value = (.MODEL_1.Motor_rl.MARKER_314)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlR2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.i_marker  &
   object_value = (.MODEL_1.spring_damper_rlCL.MARKER_315)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.j_marker  &
   object_value = (.MODEL_1.Motor_rl.MARKER_316)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R2)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rlR3.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlR3
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.i_marker  &
   object_value = (.MODEL_1.spring_damper_flSL1.MARKER_317)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.j_marker  &
   object_value = (.MODEL_1.Motor_fl.MARKER_318)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flR1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.i_marker  &
   object_value = (.MODEL_1.spring_damper_flSL2.MARKER_319)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.j_marker  &
   object_value = (.MODEL_1.Motor_fl.MARKER_320)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flR2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.i_marker  &
   object_value = (.MODEL_1.spring_damper_flCL.MARKER_321)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.j_marker  &
   object_value = (.MODEL_1.Motor_fl.MARKER_322)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R2)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_flR3.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flR3
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.i_marker  &
   object_value = (.MODEL_1.spring_damper_frSL2.MARKER_323)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.j_marker  &
   object_value = (.MODEL_1.Motor_fr.MARKER_324)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frR1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.i_marker  &
   object_value = (.MODEL_1.spring_damper_frSL1.MARKER_325)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.j_marker  &
   object_value = (.MODEL_1.Motor_fr.MARKER_326)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frR2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.i_marker  &
   object_value = (.MODEL_1.spring_damper_frCL.MARKER_327)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.j_marker  &
   object_value = (.MODEL_1.Motor_fr.MARKER_328)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R2)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_frR3.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frR3
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.i_marker  &
   object_value = (.MODEL_1.spring_damper_rrSL2.MARKER_329)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.j_marker  &
   object_value = (.MODEL_1.Motor_rr.MARKER_330)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrR1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.i_marker  &
   object_value = (.MODEL_1.spring_damper_rrSL1.MARKER_331)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.j_marker  &
   object_value = (.MODEL_1.Motor_rr.MARKER_332)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrR2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.i_marker  &
   object_value = (.MODEL_1.spring_damper_rrCL.MARKER_333)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.j_marker  &
   object_value = (.MODEL_1.Motor_rr.MARKER_334)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.stiffness_coefficient  &
   real_value = (.MODEL_1.K_R2)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_rrR3.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrR3
!
undo end_block
!
!--------------------------- UDE Dependent Objects ----------------------------!
!
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_52  &
   adams_id = 52  &
   i_marker_name = .MODEL_1.Motor_fl.MARKER_356  &
   j_marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_357
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_52  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_53  &
   adams_id = 53  &
   i_marker_name = .MODEL_1.Motor_fr.MARKER_358  &
   j_marker_name = .MODEL_1.Tire_fr.wheel_part.MARKER_359
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_53  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_54  &
   adams_id = 54  &
   i_marker_name = .MODEL_1.Motor_rl.MARKER_360  &
   j_marker_name = .MODEL_1.Tire_rl.wheel_part.MARKER_361
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_54  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_55  &
   adams_id = 55  &
   i_marker_name = .MODEL_1.Motor_rr.MARKER_362  &
   j_marker_name = .MODEL_1.Tire_rr.wheel_part.MARKER_363
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_55  &
   name_visibility = off
!
force create direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_25  &
   adams_id = 25  &
   type_of_freedom = rotational  &
   i_marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_364  &
   j_marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_365  &
   action_only = on  &
   function = ""
!
force create direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_26  &
   adams_id = 26  &
   type_of_freedom = rotational  &
   i_marker_name = .MODEL_1.Tire_fr.wheel_part.MARKER_366  &
   j_marker_name = .MODEL_1.Tire_fr.wheel_part.MARKER_367  &
   action_only = on  &
   function = ""
!
force create direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_27  &
   adams_id = 27  &
   type_of_freedom = rotational  &
   i_marker_name = .MODEL_1.Tire_rl.wheel_part.MARKER_368  &
   j_marker_name = .MODEL_1.Tire_rl.wheel_part.MARKER_369  &
   action_only = on  &
   function = ""
!
force create direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_28  &
   adams_id = 28  &
   type_of_freedom = rotational  &
   i_marker_name = .MODEL_1.Tire_rr.wheel_part.MARKER_370  &
   j_marker_name = .MODEL_1.Tire_rr.wheel_part.MARKER_371  &
   action_only = on  &
   function = ""
!
!------------------------------ Dynamic Graphics ------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
geometry create shape force  &
   force_name = .MODEL_1.SFORCE_25_force_graphic_1  &
   adams_id = 300  &
   force_element_name = .MODEL_1.SFORCE_25  &
   applied_at_marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_364
!
geometry create shape force  &
   force_name = .MODEL_1.SFORCE_26_force_graphic_1  &
   adams_id = 301  &
   force_element_name = .MODEL_1.SFORCE_26  &
   applied_at_marker_name = .MODEL_1.Tire_fr.wheel_part.MARKER_366
!
geometry create shape force  &
   force_name = .MODEL_1.SFORCE_27_force_graphic_1  &
   adams_id = 302  &
   force_element_name = .MODEL_1.SFORCE_27  &
   applied_at_marker_name = .MODEL_1.Tire_rl.wheel_part.MARKER_368
!
geometry create shape force  &
   force_name = .MODEL_1.SFORCE_28_force_graphic_1  &
   adams_id = 303  &
   force_element_name = .MODEL_1.SFORCE_28  &
   applied_at_marker_name = .MODEL_1.Tire_rr.wheel_part.MARKER_370
!
!---------------------------------- Motions -----------------------------------!
!
!
constraint create motion_generator  &
   motion_name = .MODEL_1.Steer_fl  &
   adams_id = 6  &
   type_of_freedom = rotational  &
   joint_name = .MODEL_1.steering_fl  &
   function = ""
!
constraint attributes  &
   constraint_name = .MODEL_1.Steer_fl  &
   name_visibility = off
!
constraint create motion_generator  &
   motion_name = .MODEL_1.Steer_rl  &
   adams_id = 7  &
   type_of_freedom = rotational  &
   joint_name = .MODEL_1.steering_rl  &
   function = ""
!
constraint attributes  &
   constraint_name = .MODEL_1.Steer_rl  &
   name_visibility = off
!
constraint create motion_generator  &
   motion_name = .MODEL_1.Steer_rr  &
   adams_id = 4  &
   type_of_freedom = rotational  &
   joint_name = .MODEL_1.steering_rr  &
   function = ""
!
constraint attributes  &
   constraint_name = .MODEL_1.Steer_rr  &
   name_visibility = off
!
constraint create motion_generator  &
   motion_name = .MODEL_1.Steer_fr  &
   adams_id = 5  &
   type_of_freedom = rotational  &
   joint_name = .MODEL_1.steering_fr  &
   function = ""
!
constraint attributes  &
   constraint_name = .MODEL_1.Steer_fr  &
   name_visibility = off
!
!---------------------------------- Accgrav -----------------------------------!
!
!
force create body gravitational  &
   gravity_field_name = gravity  &
   y_component_gravity = -9.80665
!
force attributes  &
   force_name = .MODEL_1.gravity  &
   visibility = off
!
!----------------------------- Analysis settings ------------------------------!
!
!
!---------------------------- Function definitions ----------------------------!
!
!
constraint modify motion_generator  &
   motion_name = .MODEL_1.Steer_fl  &
   function = "VARVAL(.MODEL_1.SteeringAngle_fl)"
!
constraint modify motion_generator  &
   motion_name = .MODEL_1.Steer_rl  &
   function = "VARVAL(.MODEL_1.SteeringAngle_rl)"
!
constraint modify motion_generator  &
   motion_name = .MODEL_1.Steer_rr  &
   function = "VARVAL(.MODEL_1.SteeringAngle_rr)"
!
constraint modify motion_generator  &
   motion_name = .MODEL_1.Steer_fr  &
   function = "VARVAL(.MODEL_1.SteeringAngle_fr)"
!
data_element modify variable  &
   variable_name = .MODEL_1.SteeringAngle_fl  &
   function = "0.5*sin(Time)"
!
data_element modify variable  &
   variable_name = .MODEL_1.SteeringAngle_fr  &
   function = "0.5*sin(Time)"
!
data_element modify variable  &
   variable_name = .MODEL_1.SteeringAngle_rl  &
   function = "0.5*sin(Time)"
!
data_element modify variable  &
   variable_name = .MODEL_1.SteeringAngle_rr  &
   function = "0.5*sin(Time)"
!
data_element modify variable  &
   variable_name = .MODEL_1.DrvTrq_fl  &
   function = "7"
!
data_element modify variable  &
   variable_name = .MODEL_1.DrvTrq_fr  &
   function = "7"
!
data_element modify variable  &
   variable_name = .MODEL_1.DrvTrq_rl  &
   function = "7"
!
data_element modify variable  &
   variable_name = .MODEL_1.DrvTrq_rr  &
   function = "7"
!
force modify direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_25  &
   function = "-VARVAL(.MODEL_1.DrvTrq_fl)"
!
force modify direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_26  &
   function = "-VARVAL(.MODEL_1.DrvTrq_fr)"
!
force modify direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_27  &
   function = "-VARVAL(.MODEL_1.DrvTrq_rl)"
!
force modify direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_28  &
   function = "-VARVAL(.MODEL_1.DrvTrq_rr)"
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.Road
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.Tire_fl
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.Tire_rr
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.Tire_rl
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.Tire_fr
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlR1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlR2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rlR3
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flR1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flR2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_flR3
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frR1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frR2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_frR3
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrR1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrR2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_rrR3
!
!------------------------- Part IC Reference Markers --------------------------!
!
!
part modify rigid_body initial_velocity  &
   part_name = .MODEL_1.Tire_fl.wheel_part  &
   vm = .MODEL_1.Tire_fl.wheel_part.wheel_cm  &
   wm = .MODEL_1.Tire_fl.wheel_part.wheel_cm
!
part modify rigid_body initial_velocity  &
   part_name = .MODEL_1.Tire_rr.wheel_part  &
   vm = .MODEL_1.Tire_rr.wheel_part.wheel_cm  &
   wm = .MODEL_1.Tire_rr.wheel_part.wheel_cm
!
part modify rigid_body initial_velocity  &
   part_name = .MODEL_1.Tire_rl.wheel_part  &
   vm = .MODEL_1.Tire_rl.wheel_part.wheel_cm  &
   wm = .MODEL_1.Tire_rl.wheel_part.wheel_cm
!
part modify rigid_body initial_velocity  &
   part_name = .MODEL_1.Tire_fr.wheel_part  &
   vm = .MODEL_1.Tire_fr.wheel_part.wheel_cm  &
   wm = .MODEL_1.Tire_fr.wheel_part.wheel_cm
!
!--------------------------- Expression definitions ---------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = ground
!
marker modify  &
   marker_name = .MODEL_1.ground.Road_ref_1  &
   location =   &
      (.MODEL_1.Road.location)  &
   orientation =   &
      (.MODEL_1.Road.orientation)
!
material modify  &
   material_name = .MODEL_1.steel  &
   youngs_modulus = (2.07E+011(Newton/meter**2))  &
   density = (7801.0(kg/meter**3))
!
material modify  &
   material_name = .MODEL_1.aluminum  &
   youngs_modulus = (7.1705E+010(Newton/meter**2))  &
   density = (2740.0(kg/meter**3))
!
material modify  &
   material_name = .MODEL_1.wood  &
   youngs_modulus = (1.1E+010(Newton/meter**2))  &
   density = (438.0(kg/meter**3))
!
material modify  &
   material_name = .MODEL_1.stainless  &
   youngs_modulus = (1.9E+011(Newton/meter**2))  &
   density = (7750.0(kg/meter**3))
!
geometry modify shape block  &
   block_name = .MODEL_1.Chasis.BOX_1  &
   diag_corner_coords =   &
      (1meter),  &
      (-1.0meter),  &
      (2.0meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Chasis.PLATE_167  &
   width = (5.0cm)  &
   radius = (10.0cm)
!
geometry modify shape block  &
   block_name = .MODEL_1.Chasis.BOX_167  &
   diag_corner_coords =   &
      (0.9meter),  &
      (-0.4meter),  &
      (-2.5meter)
!
part modify rigid_body mass_properties  &
   part_name = .MODEL_1.Chasis  &
   density = (100.0(kg/meter**3))
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.Motor_fl.CYLINDER_47  &
   length = (0.18meter)  &
   radius = (7.5E-002meter)
!
geometry modify shape link  &
   link_name = .MODEL_1.Motor_fl.LINK_58  &
   width = (3.0E-002meter)  &
   depth = (2.0E-002meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Motor_fl.PLATE_58  &
   width = (2.0cm)  &
   radius = (2.0cm)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.Motor_rr.CYLINDER_47  &
   length = (0.18meter)  &
   radius = (7.5E-002meter)
!
geometry modify shape link  &
   link_name = .MODEL_1.Motor_rr.LINK_58  &
   width = (3.0E-002meter)  &
   depth = (2.0E-002meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Motor_rr.PLATE_58  &
   width = (2.0cm)  &
   radius = (2.0cm)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.Susp_fl.CYLINDER_9  &
   length = (0.1meter)  &
   radius = (5.0E-002meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Susp_fl.PLATE_59  &
   width = (2.0cm)  &
   radius = (5.0cm)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flSL1.CYLINDER_56  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flSU1.CYLINDER_53  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.Susp_rr.CYLINDER_9  &
   length = (0.1meter)  &
   radius = (5.0E-002meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Susp_rr.PLATE_59  &
   width = (2.0cm)  &
   radius = (5.0cm)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flSL2.CYLINDER_50  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flSU2.CYLINDER_54  &
   length = (0.2meter)  &
   radius = (2.55E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrSU1.CYLINDER_53  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrSL1.CYLINDER_56  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrSU2.CYLINDER_54  &
   length = (0.2meter)  &
   radius = (2.55E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrSL2.CYLINDER_50  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrCU.CYLINDER_55  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rrCL.CYLINDER_57  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.Motor_fr.CYLINDER_47  &
   length = (0.18meter)  &
   radius = (7.5E-002meter)
!
geometry modify shape link  &
   link_name = .MODEL_1.Motor_fr.LINK_58  &
   width = (3.0E-002meter)  &
   depth = (2.0E-002meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Motor_fr.PLATE_58  &
   width = (2.0cm)  &
   radius = (2.0cm)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.Susp_fr.CYLINDER_9  &
   length = (0.1meter)  &
   radius = (5.0E-002meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Susp_fr.PLATE_59  &
   width = (2.0cm)  &
   radius = (5.0cm)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frSL1.CYLINDER_56  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frSU1.CYLINDER_53  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frSL2.CYLINDER_50  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frSU2.CYLINDER_54  &
   length = (0.2meter)  &
   radius = (2.55E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frCU.CYLINDER_55  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_frCL.CYLINDER_57  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flCU.CYLINDER_55  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flCL.CYLINDER_57  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.Motor_rl.CYLINDER_47  &
   length = (0.18meter)  &
   radius = (7.5E-002meter)
!
geometry modify shape link  &
   link_name = .MODEL_1.Motor_rl.LINK_58  &
   width = (3.0E-002meter)  &
   depth = (2.0E-002meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Motor_rl.PLATE_58  &
   width = (2.0cm)  &
   radius = (2.0cm)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.Susp_rl.CYLINDER_9  &
   length = (0.1meter)  &
   radius = (5.0E-002meter)
!
geometry modify shape plate  &
   plate_name = .MODEL_1.Susp_rl.PLATE_59  &
   width = (2.0cm)  &
   radius = (5.0cm)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlSL1.CYLINDER_56  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlSU1.CYLINDER_53  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlSL2.CYLINDER_50  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlSU2.CYLINDER_54  &
   length = (0.2meter)  &
   radius = (2.55E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlCU.CYLINDER_55  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_rlCL.CYLINDER_57  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
variable modify  &
   variable_name = .MODEL_1.SideSpringK  &
   real_value = (1.1E+005(newton/meter))
!
variable modify  &
   variable_name = .MODEL_1.SideSpringD  &
   real_value = (200(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.CentralSpringK  &
   real_value = (1.5E+004(newton/meter))
!
variable modify  &
   variable_name = .MODEL_1.CentralSpringD  &
   real_value = (2000(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.K_R1  &
   real_value = (2000.0(newton/meter))
!
variable modify  &
   variable_name = .MODEL_1.K_R2  &
   real_value = (2.0E+004(newton/meter))
!
geometry modify shape force  &
   force_name = .MODEL_1.SFORCE_25_force_graphic_1  &
   applied_at_marker_name = (.MODEL_1.SFORCE_25.i)
!
geometry modify shape force  &
   force_name = .MODEL_1.SFORCE_26_force_graphic_1  &
   applied_at_marker_name = (.MODEL_1.SFORCE_26.i)
!
geometry modify shape force  &
   force_name = .MODEL_1.SFORCE_27_force_graphic_1  &
   applied_at_marker_name = (.MODEL_1.SFORCE_27.i)
!
geometry modify shape force  &
   force_name = .MODEL_1.SFORCE_28_force_graphic_1  &
   applied_at_marker_name = (.MODEL_1.SFORCE_28.i)
!
model display  &
   model_name = MODEL_1
