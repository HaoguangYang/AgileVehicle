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
   icon_visibility = on  &
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
   variable_name = .MODEL_1.TraX  &
   adams_id = 1  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.TraY  &
   adams_id = 2  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.DrvRot  &
   adams_id = 3  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.DrvTrq  &
   adams_id = 4  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.Steer  &
   adams_id = 5  &
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
   marker_name = .MODEL_1.ground.MARKER_309  &
   adams_id = 309  &
   location = 0.55, 1.35, 1.15  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.ground.Road_ref_1  &
   adams_id = 30  &
   location = -0.7, 0.0, 4.55  &
   orientation = -90.0d, 0.0d, 90.0d
!
marker attributes  &
   marker_name = .MODEL_1.ground.Road_ref_1  &
   visibility = off
!
! ****** Floating Markers for current part ******
!
floating_marker create  &
   floating_marker_name = .MODEL_1.ground.Tire_fl_tire_jf_1  &
   adams_id = 318
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.ground  &
   material_type = .MODEL_1.steel
!
part attributes  &
   part_name = .MODEL_1.ground  &
   name_visibility = off
!
!---------------------------------- PART_45 -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.PART_45  &
   adams_id = 45  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.PART_45
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.PART_45.MARKER_291  &
   adams_id = 291  &
   location = 1.0, 0.8, 1.2  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.PART_45.cm  &
   adams_id = 296  &
   location = 0.75, 1.05, 0.95  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.PART_45.MARKER_293  &
   adams_id = 293  &
   location = 0.75, 0.8, 1.0  &
   orientation = 90.0d, 0.0d, 142.0758619484d
!
marker create  &
   marker_name = .MODEL_1.PART_45.MARKER_297  &
   adams_id = 297  &
   location = 0.75, 0.8, 1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.PART_45  &
   density = 1500.0
!
! ****** Graphics for current part ******
!
geometry create shape block  &
   block_name = .MODEL_1.PART_45.BOX_222  &
   adams_id = 222  &
   corner_marker = .MODEL_1.PART_45.MARKER_291  &
   diag_corner_coords = -0.5, 0.5, 0.5
!
part attributes  &
   part_name = .MODEL_1.PART_45  &
   color = GREEN  &
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
   marker_name = .MODEL_1.Motor_fl.MARKER_300  &
   adams_id = 300  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, 0.0d, -180.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_302  &
   adams_id = 302  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, 0.0d, -180.0d
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
   marker_name = .MODEL_1.Motor_fl.MARKER_306  &
   adams_id = 306  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
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
   marker_name = .MODEL_1.Motor_fl.MARKER_312  &
   adams_id = 312  &
   location = -0.75, 0.3, -1.0  &
   orientation = 0.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Motor_fl.MARKER_313  &
   adams_id = 313  &
   location = -0.75, 0.3, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
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
!---------------------------------- PART_46 -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.PART_46  &
   adams_id = 46  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.PART_46
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.PART_46.MARKER_298  &
   adams_id = 298  &
   location = 0.75, 0.8, 1.0  &
   orientation = -90.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.PART_46.MARKER_307  &
   adams_id = 307  &
   location = 0.5, 1.3, 1.2  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.PART_46.cm  &
   adams_id = 310  &
   location = 0.55, 1.35, 1.15  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.PART_46.MARKER_308  &
   adams_id = 308  &
   location = 0.55, 1.35, 1.15  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.PART_46  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
geometry create shape block  &
   block_name = .MODEL_1.PART_46.BOX_235  &
   adams_id = 235  &
   corner_marker = .MODEL_1.PART_46.MARKER_307  &
   diag_corner_coords = 0.1, 0.1, -0.1
!
part attributes  &
   part_name = .MODEL_1.PART_46  &
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
   marker_name = .MODEL_1.Susp_fl.MARKER_292  &
   adams_id = 292  &
   location = -0.75, 0.8, -1.0  &
   orientation = 90.0d, 0.0d, -37.9241380516d
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
marker create  &
   marker_name = .MODEL_1.Susp_fl.MARKER_314  &
   adams_id = 314  &
   location = -0.75, 0.3, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
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
   marker_name = .MODEL_1.spring_damper_flSL1.MARKER_299  &
   adams_id = 299  &
   location = -0.60345, 0.315, -0.887  &
   orientation = -180.0d, 0.0d, -180.0d
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
   marker_name = .MODEL_1.spring_damper_flSL2.MARKER_301  &
   adams_id = 301  &
   location = -0.60345, 0.315, -1.113  &
   orientation = -180.0d, 0.0d, -180.0d
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
   marker_name = .MODEL_1.spring_damper_flCL.MARKER_305  &
   adams_id = 305  &
   location = -0.50345, 0.401, -1.0  &
   orientation = -180.0d, 0.0d, -180.0d
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
!----------------------------------- Joints -----------------------------------!
!
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_54  &
   adams_id = 54  &
   i_marker_name = .MODEL_1.Susp_fl.MARKER_292  &
   j_marker_name = .MODEL_1.PART_45.MARKER_293
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_54  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .MODEL_1.JOINT_55  &
   adams_id = 55  &
   i_marker_name = .MODEL_1.PART_45.MARKER_297  &
   j_marker_name = .MODEL_1.PART_46.MARKER_298
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_55  &
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
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 1.1E+005
!
variable create  &
   variable_name = .MODEL_1.SideSpringD  &
   units = "no_units"  &
   range = 200.0, 1000.0  &
   use_allowed_values = no  &
   real_value = 974.2193469856
!
variable create  &
   variable_name = .MODEL_1.CentralSpringK  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 1.5E+004
!
variable create  &
   variable_name = .MODEL_1.CentralSpringD  &
   units = "no_units"  &
   range = 100.0, 1.0E+004  &
   use_allowed_values = no  &
   real_value = 9865.0356602541
!
variable create  &
   variable_name = .MODEL_1.SideSpringL  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 0.41
!
variable create  &
   variable_name = .MODEL_1.CentralSpringL  &
   units = "no_units"  &
   range = -10.0, 10.0  &
   use_allowed_values = no  &
   delta_type = percent_relative  &
   real_value = 0.33
!
variable create  &
   variable_name = .MODEL_1.k_R2  &
   units = "no_units"  &
   range = -1.0, 1.0  &
   use_allowed_values = no  &
   real_value = 2.0E+004
!
variable create  &
   variable_name = .MODEL_1.k_R1  &
   units = "no_units"  &
   range = -1.0, 1.0  &
   use_allowed_values = no  &
   real_value = 2000.0
!
!----------------------------- Simulation Scripts -----------------------------!
!
!
simulation script create  &
   sim_script_name = .MODEL_1.Last_Sim  &
   commands =   &
              "simulation single_run transient type=auto_select initial_static=no end_time=15.0 number_of_steps=1500 model_name=.MODEL_1"
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
   location = -0.7, 0.0, 4.55  &
   orientation = -90.0, 0.0, 90.0
!
ude attributes  &
   instance_name = .MODEL_1.Road  &
   color = DimGray  &
   visibility = off
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_RS1  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_RS1  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_RS2  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_RS2  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.SPRING_RC  &
   definition_name = .MDI.Forces.spring  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude attributes  &
   instance_name = .MODEL_1.SPRING_RC  &
   color = RED
!
ude create instance  &
   instance_name = .MODEL_1.MOTION_2  &
   definition_name = .MDI.Constraints.general_motion  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
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
   instance_name = .MODEL_1.Tire_fl  &
   definition_name = .MDI.Forces.vpg_tire  &
   location = 0.75, 0.3, 1.0  &
   orientation = 90.0, 90.0, 0.0
!
marker create  &
   marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_311  &
   adams_id = 311  &
   location = 0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_315  &
   adams_id = 315  &
   location = 0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_316  &
   adams_id = 316  &
   location = 0.75, 0.3, 1.0  &
   orientation = 0.0d, -90.0d, 0.0d
!
ude create instance  &
   instance_name = .MODEL_1.MOTION_1  &
   definition_name = .MDI.Constraints.general_motion  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
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
   string_value = "C:/MSC.Software/Adams_x64/2012/acar/shared_car_database.cdb/roads.tbl/mdi_2d_ramp.rdf"
!
variable modify  &
   variable_name = .MODEL_1.Road.road_graphics  &
   string_value = "off"
!
ude modify instance  &
   instance_name = .MODEL_1.Road
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.i_marker  &
   object_value = (.MODEL_1.spring_damper_flSL1.MARKER_299)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.j_marker  &
   object_value = (.MODEL_1.Motor_fl.MARKER_300)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.stiffness_coefficient  &
   real_value = (.MODEL_1.k_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS1.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_RS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.i_marker  &
   object_value = (.MODEL_1.spring_damper_flSL2.MARKER_301)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.j_marker  &
   object_value = (.MODEL_1.Motor_fl.MARKER_302)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.stiffness_coefficient  &
   real_value = (.MODEL_1.k_R1)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RS2.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_RS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.i_marker  &
   object_value = (.MODEL_1.spring_damper_flCL.MARKER_305)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.j_marker  &
   object_value = (.MODEL_1.Motor_fl.MARKER_306)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.stiffness_mode  &
   string_value = "linear"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.stiffness_coefficient  &
   real_value = (.MODEL_1.k_R2)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.stiffness_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.damping_mode  &
   string_value = "none"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.damping_coefficient  &
   real_value = (29.0(newton-sec/meter))
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.damping_spline  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.free_length_mode  &
   string_value = "Constant_Value"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.free_length  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.preload  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.i_dynamic_visibility  &
   string_value = "On"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.j_dynamic_visibility  &
   string_value = "Off"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.spring_visibility  &
   string_value = "depends"
!
variable modify  &
   variable_name = .MODEL_1.SPRING_RC.damper_visibility  &
   string_value = "depends"
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_RC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.i_marker  &
   object_value = .MODEL_1.PART_46.MARKER_308
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.j_marker  &
   object_value = .MODEL_1.ground.MARKER_309
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.constraint  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t1_type  &
   integer_value = 0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t2_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t3_type  &
   integer_value = 0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r1_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r2_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r3_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t1_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t2_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t3_func  &
   string_value = "5* time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r1_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r2_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r3_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t1_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t2_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t3_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r1_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r2_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r3_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t1_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t2_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.t3_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r1_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r2_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_2.r3_ic_velo  &
   real_value = 0.0
!
ude modify instance  &
   instance_name = .MODEL_1.MOTION_2
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
   object_value = .MODEL_1.ground.Tire_fl_tire_jf_1
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
   variable_name = .MODEL_1.MOTION_1.i_marker  &
   object_value = .MODEL_1.Motor_fl.MARKER_313
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.j_marker  &
   object_value = .MODEL_1.Susp_fl.MARKER_314
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.constraint  &
   object_value = (NONE)
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t1_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t2_type  &
   integer_value = 0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t3_type  &
   integer_value = 1
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r1_type  &
   integer_value = 0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r2_type  &
   integer_value = 0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r3_type  &
   integer_value = 0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t1_func  &
   string_value = "VARVAL(TraY)"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t2_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t3_func  &
   string_value = "VARVAL(TraX)"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r1_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r2_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r3_func  &
   string_value = "0 * time"
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t1_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t2_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t3_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r1_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r2_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r3_ic_disp  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t1_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t2_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.t3_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r1_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r2_ic_velo  &
   real_value = 0.0
!
variable modify  &
   variable_name = .MODEL_1.MOTION_1.r3_ic_velo  &
   real_value = 0.0
!
ude modify instance  &
   instance_name = .MODEL_1.MOTION_1
!
undo end_block
!
!--------------------------- UDE Dependent Objects ----------------------------!
!
!
constraint create joint revolute  &
   joint_name = .MODEL_1.JOINT_56  &
   adams_id = 56  &
   i_marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_311  &
   j_marker_name = .MODEL_1.Motor_fl.MARKER_312
!
constraint attributes  &
   constraint_name = .MODEL_1.JOINT_56  &
   name_visibility = off
!
force create direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_1  &
   adams_id = 1  &
   type_of_freedom = rotational  &
   i_marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_315  &
   j_marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_316  &
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
   force_name = .MODEL_1.SFORCE_1_force_graphic_1  &
   adams_id = 248  &
   force_element_name = .MODEL_1.SFORCE_1  &
   applied_at_marker_name = .MODEL_1.Tire_fl.wheel_part.MARKER_315
!
!---------------------------------- Motions -----------------------------------!
!
!
constraint create motion_generator  &
   motion_name = .MODEL_1.MOTION_7  &
   adams_id = 7  &
   type_of_freedom = rotational  &
   joint_name = .MODEL_1.JOINT_54  &
   function = ""
!
constraint attributes  &
   constraint_name = .MODEL_1.MOTION_7  &
   name_visibility = off
!
constraint create motion_generator  &
   motion_name = .MODEL_1.MOTION_8  &
   adams_id = 8  &
   type_of_freedom = rotational  &
   joint_name = .MODEL_1.JOINT_56  &
   time_derivative = velocity  &
   function = ""
!
constraint attributes  &
   constraint_name = .MODEL_1.MOTION_8  &
   active = off  &
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
!---------------------------- Adams/View Functions ----------------------------!
!
!
function create  &
   function_name = .sout  &
   text_of_expression = "STR_SPRINTF(P1, P2)"  &
   argument_names = "P1", "P2"  &
   type = string
!
!---------------------------- Function definitions ----------------------------!
!
!
constraint modify motion_generator  &
   motion_name = .MODEL_1.MOTION_7  &
   function = "VARVAL(.MODEL_1.Steer)"
!
constraint modify motion_generator  &
   motion_name = .MODEL_1.MOTION_8  &
   function = "-VARVAL(.MODEL_1.DrvRot)"
!
data_element modify variable  &
   variable_name = .MODEL_1.TraX  &
   function = "0"
!
data_element modify variable  &
   variable_name = .MODEL_1.TraY  &
   function = "0"
!
data_element modify variable  &
   variable_name = .MODEL_1.DrvRot  &
   function = "0"
!
data_element modify variable  &
   variable_name = .MODEL_1.DrvTrq  &
   function = "7"
!
data_element modify variable  &
   variable_name = .MODEL_1.Steer  &
   function = "0"
!
force modify direct single_component_force  &
   single_component_force_name = .MODEL_1.SFORCE_1  &
   function = "-VARVAL(.MODEL_1.DrvTrq)"
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
   instance_name = .MODEL_1.SPRING_RS1
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_RS2
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.SPRING_RC
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.MOTION_2
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
   instance_name = .MODEL_1.Tire_fl
!
!-------------------------- Adams/View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.MOTION_1
!
!------------------------- Part IC Reference Markers --------------------------!
!
!
part modify rigid_body initial_velocity  &
   part_name = .MODEL_1.Tire_fl.wheel_part  &
   vm = .MODEL_1.Tire_fl.wheel_part.wheel_cm  &
   wm = .MODEL_1.Tire_fl.wheel_part.wheel_cm
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
   block_name = .MODEL_1.PART_45.BOX_222  &
   diag_corner_coords =   &
      (-0.5meter),  &
      (0.5meter),  &
      (0.5meter)
!
part modify rigid_body mass_properties  &
   part_name = .MODEL_1.PART_45  &
   density = (1500.0(kg/meter**3))
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
geometry modify shape block  &
   block_name = .MODEL_1.PART_46.BOX_235  &
   diag_corner_coords =   &
      (0.1meter),  &
      (0.1meter),  &
      (-0.1meter)
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
   cylinder_name = .MODEL_1.spring_damper_flCU.CYLINDER_55  &
   length = (0.2meter)  &
   radius = (2.5E-002meter)
!
geometry modify shape cylinder  &
   cylinder_name = .MODEL_1.spring_damper_flCL.CYLINDER_57  &
   length = (0.2meter)  &
   radius = (1.5E-002meter)
!
variable modify  &
   variable_name = .MODEL_1.SideSpringK  &
   real_value = (1.1E+005(newton/meter))
!
variable modify  &
   variable_name = .MODEL_1.CentralSpringK  &
   real_value = (1.5E+004(newton/meter))
!
geometry modify shape force  &
   force_name = .MODEL_1.SFORCE_1_force_graphic_1  &
   applied_at_marker_name = (.MODEL_1.SFORCE_1.i)
!
model display  &
   model_name = MODEL_1
