//
// Created by shipei on 6/23/17.
//
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <fcntl.h>
#include <RobotCommander.h>

double MAX_SPEED = 1.0;
double MAX_SPIN_RATE = 0.8;
double MAX_ACC = 3.0;
double MAX_SPIN_ACC = 2.4;
double ACC_JERK = 10.0;
double UPDATE_SPEED = 0.01;
double KEY_TIMEOUT = 0.4;
double STOP_DEACC = 1.5;

double current_speed = 0;
double current_spin_rate = 0;
double current_acc = 0;
double current_spin_acc = 0;
int spin_dir;
int move_dir;
bool brake = false;

ros::Publisher twist_commander;

// goto xy and kbhit in Linux
void gotoxy(int x,int y)
{
    printf("%c[%d;%df",0x1B,y,x);
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void updateCallback(const ros::TimerEvent &event) {
    // calculate speed and acc, using acc jerk
    if (spin_dir == LEFT) {
        if (current_spin_acc < MAX_SPIN_ACC) {
            current_spin_acc += ACC_JERK * UPDATE_SPEED;
        } else {
            current_spin_acc = MAX_SPIN_ACC;
        }
        if (current_spin_rate < MAX_SPIN_RATE) {
            current_spin_rate += current_spin_acc * UPDATE_SPEED;
        } else {
            current_spin_rate = MAX_SPIN_RATE;
        }
    } else if (spin_dir == RIGHT) {
        if (current_spin_acc > -MAX_SPIN_ACC) {
            current_spin_acc += -1 * ACC_JERK * UPDATE_SPEED;
        } else {
            current_spin_acc = -MAX_SPIN_ACC;
        }
        if (current_spin_rate > -MAX_SPIN_RATE) {
            current_spin_rate += current_spin_acc * UPDATE_SPEED;
        } else {
            current_spin_rate = -MAX_SPIN_RATE;
        }
    } else if (spin_dir == NONE) {
        // use stop deceleration, no jerk
        current_spin_acc = 0;
        if (current_spin_rate > STOP_DEACC * UPDATE_SPEED) {
            current_spin_rate -= STOP_DEACC * UPDATE_SPEED;
        } else if (current_spin_rate > 0) {
            current_spin_rate = 0;
        } else if (current_spin_rate < -STOP_DEACC * UPDATE_SPEED) {
            current_spin_rate -= -STOP_DEACC * UPDATE_SPEED;
        } else {
            current_spin_rate = 0;
        }
    }
    // same for forward
    if (move_dir == FORWARD) {
        if (current_acc < MAX_ACC) {
            current_acc += ACC_JERK * UPDATE_SPEED;
        } else {
            current_acc = MAX_ACC;
        }
        if (current_speed < MAX_SPEED) {
            current_speed += current_acc * UPDATE_SPEED;
        } else {
            current_speed = MAX_SPEED;
        }
    } else if (move_dir == BACKWARD) {
        if (current_acc > -MAX_ACC) {
            current_acc += -ACC_JERK * UPDATE_SPEED;
        } else {
            current_acc = -MAX_ACC;
        }
        if (current_speed > -MAX_SPEED) {
            current_speed += current_acc * UPDATE_SPEED;
        } else {
            current_speed = -MAX_SPEED;
        }
    } else if (move_dir == NONE) {
        // use stop deceleration, no jerk
        current_acc = 0;
        if (current_speed > STOP_DEACC * UPDATE_SPEED) {
            current_speed -= STOP_DEACC * UPDATE_SPEED;
        } else if (current_speed > 0) {
            current_speed = 0;
        } else if (current_speed < -STOP_DEACC * UPDATE_SPEED) {
            current_speed -= -STOP_DEACC * UPDATE_SPEED;
        } else {
            current_speed = 0;
        }
    }
//    ROS_INFO_THROTTLE(0.5, "ACC: %f, SPEED: %f, SPIN RATE: %f, SPIN_ACC: %f", current_acc, current_speed, current_spin_rate, current_spin_acc);
    geometry_msgs::Twist twist;
    if (brake) {
        brake = false;
        move_dir = NONE;
        spin_dir = NONE;
        current_speed = 0;
        current_spin_rate = 0;
        current_acc = 0;
        current_spin_acc = 0;
    } else {
        twist.angular.z = current_spin_rate;
        twist.linear.x = current_speed;
    }
    twist_commander.publish(twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_controller"); //name this node
    ros::NodeHandle nh;
    nh.param("MAX_SPEED", MAX_SPEED, MAX_SPEED);
    nh.param("MAX_SPIN_RATE", MAX_SPIN_RATE, MAX_SPIN_RATE);
    nh.param("MAX_ACC", MAX_ACC, MAX_ACC);
    nh.param("MAX_SPIN_ACC", MAX_SPIN_ACC, MAX_SPIN_ACC);
    nh.param("ACC_JERK", ACC_JERK, ACC_JERK);
    nh.param("UPDATE_SPEED", UPDATE_SPEED, UPDATE_SPEED);
    nh.param("KEY_TIMEOUT", KEY_TIMEOUT, KEY_TIMEOUT);
    nh.param("STOP_DEACC", STOP_DEACC, STOP_DEACC);
    ros::Timer update_timer = nh.createTimer(ros::Duration(UPDATE_SPEED), updateCallback);
    twist_commander = nh.advertise<geometry_msgs::Twist>("key_vel", 10);
    ros::AsyncSpinner spinner(1); // for timer callback to execute
    spinner.start();
    std::system("clear");
    std::cout   << "=============Simple keyboard teleop program=============" << std::endl
                << "Control the mobile robot with WASD just like in need4speed" << std::endl
                << "Use param server to set parameters like speed, acceleration, jerk, etc." << std::endl
                << std::endl
                << "         ↑" << std::endl
                << "         W " << std::endl
                << "      ←A S D→" << std::endl
                << "         ↓  " << std::endl
                << std::endl
                << "  Q = Quit, E = Brake" << std::endl
                << std::endl
                << "Status: " << std::endl;
    auto move_time = ros::Time::now();
    auto spin_time = ros::Time::now();
    while (ros::ok()) {
        if (kbhit()) {
            int key = getchar();
            switch (key) {
                case 'A': case 'a':
                    gotoxy(9, 12);
                    std::cout << "Go left!              " << std::endl;
                    spin_dir = LEFT;
                    spin_time = ros::Time::now();
                    break;
                case 'S': case 's':
                    gotoxy(9, 12);
                    std::cout << "Go back!              " << std::endl;
                    move_dir = BACKWARD;
                    move_time = ros::Time::now();
                    break;
                case 'D': case 'd':
                    gotoxy(9, 12);
                    std::cout << "Go right!              " << std::endl;
                    spin_dir = RIGHT;
                    spin_time = ros::Time::now();
                    break;
                case 'W': case 'w':
                    gotoxy(9, 12);
                    std::cout << "Go ahead!              " << std::endl;
                    move_dir = FORWARD;
                    move_time = ros::Time::now();
                    break;
                case 'E': case 'e':
                    brake = true;
                    gotoxy(9, 12);
                    std::cout << "Brake!              " << std::endl;
                    move_dir = NONE;
                    spin_dir = NONE;
                    break;
                case 'Q': case 'q':
                    gotoxy(9, 12);
                    std::cout << "Quit!              " << std::endl;
                    return 0;
                default:
                    gotoxy(9, 12);
                    std::cout << "Wrong key pressed!" << std::endl;
            }
        }
        if ((ros::Time::now() - spin_time).toSec() > KEY_TIMEOUT && spin_dir != NONE) {
            spin_dir = NONE;
            gotoxy(9, 12);
            std::cout << "Stop spin              " << std::endl;
        }
        if ((ros::Time::now() - move_time).toSec() > KEY_TIMEOUT && move_dir != NONE) {
            move_dir = NONE;
            gotoxy(9, 12);
            std::cout << "Stop move              " << std::endl;
        }
    }
    return 0;
}
