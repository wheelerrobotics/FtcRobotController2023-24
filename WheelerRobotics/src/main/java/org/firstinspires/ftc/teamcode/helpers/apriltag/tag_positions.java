package org.firstinspires.ftc.teamcode.helpers.apriltag;

public class tag_positions {
    // all angles in radians
    // all lengths in inches


    public static double board_tag_z = (.1039230485);
    public static double wall_tag_z  = (1); // find this


public static final
    global_position left_tag_red = setting_global_position.set_global_position(29.5 , 60 , board_tag_z , 0.52359877559 , 0 , 0 , 4);
public static final
    global_position center_tag_red = setting_global_position.set_global_position(35.5 ,60 ,board_tag_z , 0.52359877559 , 0 , 0 , 5);
public static final
    global_position right_tag_red = setting_global_position.set_global_position(41.5 , 60 , board_tag_z , 0.52359877559 , 0 , 0 , 6);
public static final
    global_position left_tag_blue = setting_global_position.set_global_position(-41.5 , 60 , board_tag_z , 0.52359877559 , 0 , 0 , 1);
public static final
    global_position center_tag_blue = setting_global_position.set_global_position(-35.5 , 60 , board_tag_z , 0.52359877559 , 0 , 0  ,2);
public static final
    global_position right_tag_blue = setting_global_position.set_global_position(-29.5 , 60 , board_tag_z , 0.52359877559 , 0 , 0 , 3);
public static final
    global_position wall_tag_left = setting_global_position.set_global_position(-5 , -60 , wall_tag_z , 0.52359877559 , 0 , 0 , 7);
public static final
    global_position wall_tag_right = setting_global_position.set_global_position(-5 , -60 , wall_tag_z , 0.52359877559 , 0 , 0 , 8);



   /* tags here
    *       -----------------------
    *       |          |y         |
    *       |          |          |
    *       |__________|__________|
    *       |          |     x    |
    *       |          |          |
    *       |          |          |
    *       -----------------------
    *
    *       0,0 is where x and y meet (yes I know lots of stuff isnt like that but my brain doesnt work like that)
    */



}
