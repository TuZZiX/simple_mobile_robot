//
// Created by shipei on 6/23/17.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <fcntl.h>
#include <RobotCommander.h>

class LinuxConsole {
public:
    LinuxConsole() {
        default_x = 0;
        default_y = 0;
    }
    // gotoxy and kbhit for Linux
    void gotoxy(int x, int y) {
        printf("%c[%d;%df", 0x1B, y, x);
    }

    int kbhit(void) {
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

        if (ch != EOF) {
            ungetc(ch, stdin);
            return 1;
        }

        return 0;
    }

    void printXY(std::string content) {
        gotoxy(default_x, default_y);
        std::cout << content << std::endl;
    }

    void setDefaultXY(int x, int y) {
        default_x = x;
        default_y = y;
    }

private:
    int default_x;
    int default_y;
};

class DirectionController {
public:
    DirectionController(ros::NodeHandle &nodeHandle): nh(nodeHandle), spinner(1) {
        MAX_SPEED = 1.0;
        MAX_SPIN_RATE = 0.8;
        MAX_ACC = 3.0;
        MAX_SPIN_ACC = 2.4;
        ACC_JERK = 10.0;
        UPDATE_RATE = 100;
        STOP_DEACC = 1.5;

        current_speed = 0;
        current_spin_rate = 0;
        current_acc = 0;
        current_spin_acc = 0;
        brake = false;

        nh.param("MAX_SPEED", MAX_SPEED, MAX_SPEED);
        nh.param("MAX_SPIN_RATE", MAX_SPIN_RATE, MAX_SPIN_RATE);
        nh.param("MAX_ACC", MAX_ACC, MAX_ACC);
        nh.param("MAX_SPIN_ACC", MAX_SPIN_ACC, MAX_SPIN_ACC);
        nh.param("ACC_JERK", ACC_JERK, ACC_JERK);
        nh.param("UPDATE_RATE", UPDATE_RATE, UPDATE_RATE);
        nh.param("STOP_DEACC", STOP_DEACC, STOP_DEACC);

        dt = 1 / UPDATE_RATE;
        update_timer = nh.createTimer(ros::Duration(dt), &DirectionController::updateCallback, this);
        twist_commander = nh.advertise<geometry_msgs::Twist>("key_vel", 10);
        spinner.start();
    }

    int getMoveDir() {
        return move_dir;
    }

    int getSpinDir() {
        return spin_dir;
    }

    void setMoveDir(int direction) {
        move_dir = direction;
    }

    void setSpinDir(int direction) {
        spin_dir = direction;
    }

    void stop(){
        brake = true;
        move_dir = NONE;
        spin_dir = NONE;
    }

private:
    double MAX_SPEED;     // max speed in m/s
    double MAX_SPIN_RATE; // max spinning rate in rad/s
    double MAX_ACC;       // max acceleration on speed in m/s^2 to start and stop
    double MAX_SPIN_ACC;  // max acceleration on spinning rate in rad/s^2
    double ACC_JERK;     // max acceleration jerk for both MAX_ACC and MAX_SPIN_ACC in m/s^3 or rad/s^3
    double UPDATE_RATE;   // speed calculation and twist publishing frequency, in hz
    double STOP_DEACC;    // when key released, instead of using max acceleration, use a slower deceleration to simulate inertia
    // the final stop time from the time that key released would be KEY_TIMEOUT + current_speed / STOP_DEACC

    double dt;
    double current_speed;
    double current_spin_rate;
    double current_acc;
    double current_spin_acc;
    int spin_dir;   // LEFT or RIGHT or NONE
    int move_dir;   // FORWARD or BACKWARD or NONE
    bool brake;     // stop as fast as you can

    ros::NodeHandle nh;
    ros::Publisher twist_commander;
    ros::Timer update_timer;
    ros::AsyncSpinner spinner;

    void updateCallback(const ros::TimerEvent &event) {
        // calculate speed and acc, using acc jerk
        if (spin_dir == LEFT) {
            if (current_spin_acc < MAX_SPIN_ACC) {
                current_spin_acc += ACC_JERK * dt;
            } else {
                current_spin_acc = MAX_SPIN_ACC;
            }
            if (current_spin_rate < MAX_SPIN_RATE) {
                current_spin_rate += current_spin_acc * dt;
            } else {
                current_spin_rate = MAX_SPIN_RATE;
            }
        } else if (spin_dir == RIGHT) {
            if (current_spin_acc > -MAX_SPIN_ACC) {
                current_spin_acc += -1 * ACC_JERK * dt;
            } else {
                current_spin_acc = -MAX_SPIN_ACC;
            }
            if (current_spin_rate > -MAX_SPIN_RATE) {
                current_spin_rate += current_spin_acc * dt;
            } else {
                current_spin_rate = -MAX_SPIN_RATE;
            }
        } else if (spin_dir == NONE) {
            // use stop deceleration, infinite jerk
            current_spin_acc = 0;
            if (current_spin_rate > STOP_DEACC * dt) {
                current_spin_rate -= STOP_DEACC * dt;
            } else if (current_spin_rate > 0) {
                current_spin_rate = 0;
            } else if (current_spin_rate < -STOP_DEACC * dt) {
                current_spin_rate -= -STOP_DEACC * dt;
            } else {
                current_spin_rate = 0;
            }
        }
        // same for forward
        if (move_dir == FORWARD) {
            if (current_acc < MAX_ACC) {
                current_acc += ACC_JERK * dt;
            } else {
                current_acc = MAX_ACC;
            }
            if (current_speed < MAX_SPEED) {
                current_speed += current_acc * dt;
            } else {
                current_speed = MAX_SPEED;
            }
        } else if (move_dir == BACKWARD) {
            if (current_acc > -MAX_ACC) {
                current_acc += -ACC_JERK * dt;
            } else {
                current_acc = -MAX_ACC;
            }
            if (current_speed > -MAX_SPEED) {
                current_speed += current_acc * dt;
            } else {
                current_speed = -MAX_SPEED;
            }
        } else if (move_dir == NONE) {
            // use stop deceleration, infinite jerk
            current_acc = 0;
            if (current_speed > STOP_DEACC * dt) {
                current_speed -= STOP_DEACC * dt;
            } else if (current_speed > 0) {
                current_speed = 0;
            } else if (current_speed < -STOP_DEACC * dt) {
                current_speed -= -STOP_DEACC * dt;
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
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_controller"); //name this node
    ros::NodeHandle nh;
    DirectionController controller(nh);
    LinuxConsole console;
    double KEY_TIMEOUT = 0.4;   // continue moving for a short period after key released, also to prevent key jitter, in second
    nh.param("KEY_TIMEOUT", KEY_TIMEOUT, KEY_TIMEOUT);
    console.setDefaultXY(9, 12);
    std::system("clear");
    std::cout << "=============Simple keyboard teleop program=============" << std::endl
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
        if (console.kbhit()) {
            int key = getchar();
            switch (key) {
                case 'A':
                case 'a':
                    console.printXY("Go left!              ");
                    controller.setSpinDir(LEFT);
                    spin_time = ros::Time::now();
                    break;
                case 'S':
                case 's':
                    console.printXY("Go back!              ");
                    controller.setMoveDir(BACKWARD);
                    move_time = ros::Time::now();
                    break;
                case 'D':
                case 'd':
                    console.printXY("Go right!              ");
                    controller.setSpinDir(RIGHT);
                    spin_time = ros::Time::now();
                    break;
                case 'W':
                case 'w':
                    console.printXY("Go ahead!              ");
                    controller.setMoveDir(FORWARD);
                    move_time = ros::Time::now();
                    break;
                case 'E':
                case 'e':
                    console.printXY("Brake!              ");
                    controller.stop();
                    break;
                case 'Q':
                case 'q':
                    console.printXY("Quit!              ");
                    return 0;
                default:
                    console.printXY("Wrong key pressed!");
            }
        }
        if ((ros::Time::now() - spin_time).toSec() > KEY_TIMEOUT && controller.getSpinDir() != NONE) {
            controller.setSpinDir(NONE);
            console.printXY("Stop spin              ");
        }
        if ((ros::Time::now() - move_time).toSec() > KEY_TIMEOUT && controller.getMoveDir() != NONE) {
            controller.setMoveDir(NONE);
            console.printXY("Stop move              ");
        }
    }
    return 0;
}
