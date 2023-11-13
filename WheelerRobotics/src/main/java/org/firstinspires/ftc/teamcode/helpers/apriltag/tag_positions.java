package org.firstinspires.ftc.teamcode.helpers.apriltag;

public class tag_positions {
    // all angles in radians
    // all lengths in inches


    public static double board_tag_z = (.1039230485);
    public static double wall_tag_z  = (1); // find this


public static final
    global_position left_tag_red = setting_global_position.set_global_position(-5 , 60 , board_tag_z , 0 , 0 , 30 , 4);
public static final
    global_position center_tag_red = setting_global_position.set_global_position(-5 ,60 ,board_tag_z , 0 , 0 , 30 , 5);
public static final
    global_position right_tag_red = setting_global_position.set_global_position(-5 , 60 , board_tag_z , 0 , 0 , 30 , 6);
public static final
    global_position left_tag_blue = setting_global_position.set_global_position(-5 , 60 , board_tag_z , 0 , 0 , 30 , 1);
public static final
    global_position center_tag_blue = setting_global_position.set_global_position(-5 , 60 , board_tag_z , 0 , 0 , 30  ,2);
public static final
    global_position right_tag_blue = setting_global_position.set_global_position(-5 , 60 , board_tag_z , 0 , 0 , 30 , 3);
public static final
    global_position wall_tag_left = setting_global_position.set_global_position(-5 , -60 , wall_tag_z , 0 , 0 , 30 , 7);
public static final
    global_position wall_tag_right = setting_global_position.set_global_position(-5 , -60 , wall_tag_z , 0 , 0 , 30 , 8);



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
