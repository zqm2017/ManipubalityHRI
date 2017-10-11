#! /usr/bin/python
import sys

def printCylinderLink(link_name,x,y,z,r,p,yy,rad_cyl,len_cyl,rc,gc,bc,ac,color_name):
    file = open("gen_arm.urdf","a")
    file.write("    <link name=\"%s\"> \n" %(link_name))
    file.write("        <visual>\n")
    file.write("            <origin xyz=\"%.2f %.2f %.2f\" rpy=\"%.2f %.2f %.2f\"/> \n" %(x,y,z,r,p,yy))
    file.write("            <geometry>\n")
    file.write("                <cylinder radius=\"%.2f\" length=\"%.2f\"/>  \n" %(rad_cyl,len_cyl))
    file.write("            </geometry> \n")
    file.write("            <material name=\"%s\"> \n"%(color_name))
    file.write("                <color rgba=\"%.2f %.2f %.2f %.2f\"/> \n" %(rc,gc,bc,ac))
    file.write("            </material> \n")
    file.write("        </visual> \n")
    file.write("    </link> \n\n")

    file.close()

def printSphericalLink(link_name,x,y,z,r,p,yy,rad,rc,gc,bc,ac,color_name):
    file = open("gen_arm.urdf","a")
    file.write("    <link name=\"%s\">\n" %(link_name))
    file.write("        <visual> \n")
    file.write("            <origin xyz=\"%.2f %.2f %.2f\" rpy=\"%.2f %.2f %.2f\"/> \n" %(x,y,z,r,p,yy))
    file.write("            <geometry> \n")
    file.write("                <sphere radius=\"%.2f\"/>  \n" %(rad))
    file.write("            </geometry> \n")
    file.write("            <material name=\"%s\">\n"%(color_name))
    file.write("                <color rgba=\"%.2f %.2f %.2f %.2f\"/> \n" %(rc,gc,bc,ac))
    file.write("            </material> \n")
    file.write("        </visual> \n")
    file.write("    </link> \n\n")

    file.close()


def printBoxLink(link_name,x,y,z,r,p,yy,bx,by,bz,rc,gc,bc,ac,color_name):
    file = open("gen_arm.urdf","a")
    file.write("    <link name=\"%s\"> \n" %(link_name))
    file.write("        <visual> \n")
    file.write("            <origin xyz=\"%.2f %.2f %.2f\" rpy=\"%.2f %.2f %.2f\"/> \n" %(x,y,z,r,p,yy))
    file.write("            <geometry> \n")
    file.write("                <box size=\"%.2f %.2f %.2f\"/>  \n" %(bx,by,bz))
    file.write("            </geometry> \n")
    file.write("            <material name=\"%s\"> \n"%(color_name))
    file.write("                <color rgba=\"%.2f %.2f %.2f %.2f\"/> \n" %(rc,gc,bc,ac))
    file.write("            </material> \n")
    file.write("        </visual> \n")
    file.write("    </link> \n\n")

    file.close()    

def printRevoluteJoint(joint_name,joint_type,parent,child,x,y,z,r,p,yy,low,up,eff,vel,xx,yyy,zz):
    file = open("gen_arm.urdf","a")
    file.write("    <joint name=\"%s\" type=\"%s\"> \n"%(joint_name,joint_type))
    file.write("    <parent link=\"%s\"/> \n"%(parent))
    file.write("    <child link=\"%s\"/> \n"%(child))
    file.write("    <origin xyz=\"%.2f %.2f %.2f\" rpy=\"%.2f %.2f %.2f\"/> \n"%(x,y,z,r,p,yy))
    file.write("    <limit lower=\"%.2f\" upper=\"%.2f\" effort=\"%.2f\" velocity=\"%.2f\"/> \n"%(low,up,eff,vel))    
    file.write("    <axis xyz=\"%d %d %d\"/> \n"%(xx,yyy,zz))
    file.write("    </joint>\n\n")

def printFixedJoint(joint_name,joint_type,parent,child,x,y,z,r,p,yy):
    file = open("gen_arm.urdf","a")
    file.write("    <joint name=\"%s\" type=\"%s\"> \n"%(joint_name,joint_type))
    file.write("    <parent link=\"%s\"/> \n"%(parent))
    file.write("    <child link=\"%s\"/> \n"%(child))
    file.write("    <origin xyz=\"%.2f %.2f %.2f\" rpy=\"%.2f %.2f %.2f\"/> \n"%(x,y,z,r,p,yy))
    file.write("    </joint>\n\n")


print("Generating Arm URDF")

if len(sys.argv) > 1:

    upper_arm_len = round(float(sys.argv[1]))
    lower_arm_len = round(float(sys.argv[2]))
    shoulder_height = round(float(sys.argv[3]))
else:
    print("No Arguments given, creating URDF with default values: Upper_arm_len = 0.44, Lower_arm_len= 0.44, Shoulder_height= 0.64")
    upper_arm_len = 0.44
    lower_arm_len = 0.44
    shoulder_height = 0.64

file = open("gen_arm.urdf","w") 
file.write("<robot name=\"human_arm\">\n\n")
file.write("<!-- * * * Link Definitions * * * -->\n\n")
file.close() 

printCylinderLink('base_link',0,0,shoulder_height/2,0,0,0,0.1,shoulder_height,0,0.9,0.9,1.0,'Cyan')

printSphericalLink('left_shoulder_forward_link',0,0,0,0,0,0,0.03,255,0,0,1.0,'Red')

printSphericalLink('left_shoulder_side_link',0,0,0,0,0,0,0.03,255,0,0,1.0,'Red')

printSphericalLink('left_shoulder_up_link',0,0,0,1.57,0,0,0.03,255,0,0,1.0,'Red')

printCylinderLink('left_upper_arm_link',0,0,0,0,0,0,0.01,upper_arm_len,0.9,0.9,0.9,1.0,'Grey1')

printSphericalLink('left_elbow_link',0,0,0,0,0,1.57,0.03,255,0,0,1.0,'Red')

printCylinderLink('left_lower_arm_link',0,0,0,0,0,0,0.01,lower_arm_len,0.9,0.9,0.9,1.0,'Grey1')

printSphericalLink('left_wrist_link',0,0,0,1.57,0,0,0.03,255,0,0,1.0,'Red')

printBoxLink('left_hand_link',0,0,0,0,0,0,0.03,0.01,0.06,0.7,0.7,0.7,1.0,'Grey1')


file = open("gen_arm.urdf","a")
file.write("<!-- * * * Joint Definitions * * * -->\n")
file.close()

printRevoluteJoint('left_shoulder_forward_joint','revolute','base_link','left_shoulder_forward_link',0,0.1,shoulder_height,0,0,0,-0.6,1.0,10,3,0,0,1)

printRevoluteJoint('left_shoulder_up_joint','revolute','left_shoulder_forward_link','left_shoulder_up_link',0,0,0,0,0,0,-1.57,1.0,10,3,0,1,0)

printRevoluteJoint('left_shoulder_side_joint','revolute','left_shoulder_up_link','left_shoulder_side_link',0,0,0,0,0,0,-0.3,1.0,10,3,1,0,0)

printFixedJoint('left_upper_arm_joint','fixed','left_shoulder_side_link','left_upper_arm_link',0,0,-upper_arm_len/2,0,0,0)

printRevoluteJoint('left_elbow_joint','revolute','left_upper_arm_link','left_elbow_link',0,0,-upper_arm_len/2,0,0,0,-2.73,0,10,3,0,1,0)

printFixedJoint('left_lower_arm_joint','fixed','left_elbow_link','left_lower_arm_link',0,0,-lower_arm_len/2,0,0,0)

printRevoluteJoint('left_wrist_joint','revolute','left_lower_arm_link','left_wrist_link',0,0,-lower_arm_len/2,0,0,0,-0.77,0.77,10,3,1,0,0)

printFixedJoint('left_hand_joint','fixed','left_wrist_link','left_hand_link',0,0,-0.055,0,0,0)





file = open("gen_arm.urdf","a") 
file.write("</robot>")
file.close() 
