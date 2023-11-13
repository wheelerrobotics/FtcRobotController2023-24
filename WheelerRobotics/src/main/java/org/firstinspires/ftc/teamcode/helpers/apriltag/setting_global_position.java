package org.firstinspires.ftc.teamcode.helpers.apriltag;

public class setting_global_position {

    public static global_position set_global_position(double global_x_input , double global_y_input , double global_z_input ,
                                                      double rotation_x_input , double rotation_y_input , double rotation_z_input , double tag_id){

        global_position tag = new global_position();
        tag.global_x = global_x_input;
        tag.global_y = global_y_input;
        tag.global_z = global_z_input;
        tag.rotation_x = rotation_x_input;
        tag.rotation_y = rotation_y_input;
        tag.rotation_z = rotation_z_input;
        tag.id = tag_id;

        return tag;

    }



}
