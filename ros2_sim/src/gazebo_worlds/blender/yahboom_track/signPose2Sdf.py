poseDict = {
    "x1": [-2.72, 0.1, 0.0],
    "x2": [-0.75, 3.1, 0.0],
    "x3": [2.72, 1.13, 0.0],
    "x4": [0.28, 3.08, 0.0],
    "x5": [-0.75, 1.13, 0.0],
    "x6": [0.28, 1.13, 0.0],
    "x7": [1.72, 1.13, 0.0],
    "x8": [-1.71, 0.1, 0.0],
    "x9": [-0.75, 0.1, 0.0],
    "x10": [0.28, 0.1, 0.0],
    "x11": [0.28, -2.07, 0.0],
    "x12": [-0.75, -1.0, 0.0]
}
rotationDict = {
    "north": [0.0, 0.0, 3.14],
    "south": [0.0, 0.0, 0.0],
    "east": [0.0, 0.0, 1.57],
    "west": [0.0, 0.0, -1.57]
}

# method that reads a sdf file, and stops when it finds the string <!-- INSERT SIGNS -->
def readSdfFile(filename, signs):
    content = ""
    line = ""
    with open(filename, "r") as file:
        while(line!="</sdf>\n"):
            line = file.readline()
            if("<!-- INSERT SIGNS -->" in line):
                index = 0
                for sign in signs:
                    content += posRot2Include(sign[0], sign[1], sign[2], index)
                    index += 1
            else:
                content += line
    return content

def posRot2Include(pointName, orientation, signName, index):
    ret = ""
    ret += '\t\t<include>\n'
    ret += '\t\t  <uri>model://YahboomTrack/' + signName + '</uri>\n'
    ret += '\t\t  <pose>' + str(poseDict[pointName][0]) + ' ' + str(poseDict[pointName][1]) + ' ' + str(poseDict[pointName][2]) + ' ' + str(rotationDict[orientation][0]) + ' ' + str(rotationDict[orientation][1]) + ' ' + str(rotationDict[orientation][2]) + '</pose>\n'
    ret += '\t\t  <name>' +str(signName) + '_' + str(index) + '</name>\n'
    ret += '\t\t</include>\n'
    return ret

# change this part of the code to insert signs in the correct places
sign1 = ["x1", "east", "SignForward"]
sign2 = ["x2", "south", "SignRight"]
sign3 = ["x10", "west", "SignForward"]
sign4 = ["x11", "west", "SignRight"]
sign5 = ["x12", "north", "SignLeft"]

signs = [sign1, sign2, sign3, sign4, sign5]
# end of signs
modifiedText = readSdfFile("/home/bee/ros2_sim/src/gazebo_worlds/worlds/yahboom_track_template.sdf", signs)

with open("/home/bee/ros2_sim/src/gazebo_worlds/worlds/yahboom_track.sdf", "w") as file:
    file.write(modifiedText)