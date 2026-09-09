#include "AnglePid.h"
#include "IncrementalPid.h"
#include "PS4Controller.h"
#include "Pid.h"
#include "constants.hpp"
#include "nnct/interfaces/interfaces.hpp"
#include <Arduino.h>

using namespace nnct::interfaces;

class Steering {
    public:
        Steering(Motor* motor, IncrementalEncoder* encoder, LimitSwitch* limit_switch, AnglePID* pid, double offset_deg,
                 uint8_t ID)
            : motor(motor), encoder(encoder), limit_switch(limit_switch), pid(pid), target_degree(0), offset_degree(offset_deg),
              ID(ID) {}
        bool calibrate_zero() { // 0点合わせ
            uint32_t startTime = millis();
            this->motor->run(CALIBRATING_DUTY);
            if (this->limit_switch->active()) { // ONから始まったら、一度OFFになるまで待つ
                this->motor->run(-CALIBRATING_DUTY);
                while (this->limit_switch->active()) {
                    if (millis() - startTime > CALIBRATING_TIMEOUT_MS) {
                        this->motor->stop();
                        return false;
                    }
                    delay(1);
                }
                this->motor->stop();
                this->encoder->clear();
                this->motor->run(CALIBRATING_DUTY);
            }

            while (!this->limit_switch->active()) { // OFF→ONになるまで待つ
                if (millis() - startTime > CALIBRATING_TIMEOUT_MS) {
                    this->motor->stop();
                    return false;
                }
                delay(1);
            }
            this->motor->stop();

            // offset を encoder count に埋め込む
            int32_t offset_count =
                static_cast<int32_t>(this->offset_degree * ENCODER_RESOLUTION * STEER_GEAR_RATIO_MOTOR_TO_STEER / 360.0);
            this->encoder->setCount(offset_count);
            return true;
        }
        void   set_target(double degree) { this->target_degree = degree; }
        double get_current_degree() {
            double degree = (encoder->getCount() * 360.0 / ENCODER_RESOLUTION) / STEER_GEAR_RATIO_MOTOR_TO_STEER;

            return normalizeAngleDeg(degree);
        }
        int32_t get_encoder_count() const { return encoder->getCount(); }
        void    update(double dt) {
            double current_degree = this->get_current_degree();
            double duty           = this->pid->update(this->target_degree, current_degree, dt);
            double error          = pid->getError();

            const double dead_zone = 1.0;
            if (fabs(error) < dead_zone) {
                duty = 0.0;
                // this->pid->reset();
            }

            // static uint32_t last_print_time = 0;
            // const uint32_t  now             = millis();

            // if (now - last_print_time >= 1000) {
            //     last_print_time = now;
            //     Serial.printf("id:%d target: %.1f current: %.1f duty: %.1f\n", ID, target_degree, current_degree, duty);
            // }

            this->motor->run(duty, -1);
        }

    private:
        static double normalizeAngleDeg(double a) {
            while (a > 180.0)
                a -= 360.0;
            while (a < -180.0)
                a += 360.0;
            return a;
        }

        Motor*              motor;
        IncrementalEncoder* encoder;
        LimitSwitch*        limit_switch;
        AnglePID*           pid;

        double target_degree;
        double offset_degree;

        const uint8_t ID;

        static const uint32_t CALIBRATING_TIMEOUT_MS = 8000;
};

class Drive {
    public:
        enum class ControlMode {
            Duty,
            Speed
        };
        Drive(RobomasMotor* motor, PID* pid, uint8_t ID)
            : motor(motor), pid(pid), mode(ControlMode::Duty), target_duty(0.0), target_mm_s(0.0), ID(ID) {}

        // duty指定
        void set_target_duty(double duty) {
            mode        = ControlMode::Duty;
            target_duty = duty;
        }

        // 速度指定[mm/s]
        void set_target_mm_s(double speed_mm_s) {
            mode        = ControlMode::Speed;
            target_mm_s = speed_mm_s;
        }

        double get_current_mm_s() { // rpm -> mm/s
            double       motor_rpm = this->motor->rpm();
            const double wheel_rpm = motor_rpm / DRIVE_GEAR_RATIO;
            return wheel_rpm * 2.0 * M_PI * DRIVE_RADIUS / 60.0;
        }

        void update(double dt) {
            double drive_command;

            if (mode == ControlMode::Speed) {
                const double current_mm_s = get_current_mm_s();

                drive_command = pid->update(target_mm_s, current_mm_s, dt);

                // static uint32_t last_print_time = 0;
                // const uint32_t  now             = millis();

                // if (now - last_print_time >= 1000) {
                //     last_print_time = now;
                //     Serial.printf("id:%d target: %.1f current: %.1f duty: %.1f\n", ID, target_mm_s, current_mm_s,
                //                   drive_command);
                // }
            } else {
                drive_command = target_duty;
            }

            motor->run(drive_command);
        }

        void stop() {
            mode        = ControlMode::Duty;
            target_duty = 0.0;
            target_mm_s = 0.0;
            motor->stop();
        }

    private:
        RobomasMotor* motor;
        PID*          pid;

        ControlMode mode;
        double      target_duty;
        double      target_mm_s;

        const uint8_t ID;
};

class SwerveDrive {
    public:
        SwerveDrive(Drive* drive, Steering* steering, uint8_t ID) : drive(drive), steering(steering), ID(ID) {}
        bool init() {
            if (!this->steering->calibrate_zero()) {
                return false;
            }
            return true;
        }

        void set_target_duty(double degree, double drive_target_duty) {
            double          current_degree = steering->get_current_degree();
            OptimizedParams params         = optimizeSteerAngle(degree, current_degree);

            steering->set_target(params.degree);
            drive->set_target_duty(drive_target_duty * params.drive_dir);
        }

        void set_target_mm_s(double degree, double drive_target_mm_s) {
            double          current_degree = steering->get_current_degree();
            OptimizedParams params         = optimizeSteerAngle(degree, current_degree);

            // Serial.printf("OPT target=%.1f current=%.1f count=%ld\n", degree, current_degree, steering->get_encoder_count());

            steering->set_target(params.degree);
            drive->set_target_mm_s(drive_target_mm_s * params.drive_dir);
        }

        void stop_drive() { drive->stop(); }
        void update(double dt) {
            this->steering->update(dt);
            this->drive->update(dt);
        }

    private:
        Drive*        drive;
        Steering*     steering;
        const uint8_t ID;

        struct OptimizedParams {
                double degree;
                int8_t drive_dir;
        };

        static double normalizeAngleDeg(double a) {
            while (a > 180.0)
                a -= 360.0;
            while (a < -180.0)
                a += 360.0;
            return a;
        }
        static OptimizedParams optimizeSteerAngle(double steer_target, double currentAngleDeg) {
            double error = normalizeAngleDeg(steer_target - currentAngleDeg);

            OptimizedParams result;
            result.drive_dir = 1;

            // 90°を超えるなら、車輪を180°反転して
            // ドライブ方向を逆にする
            if (error > 90.0) {
                error -= 180.0;
                result.drive_dir = -1;
            } else if (error < -90.0) {
                error += 180.0;
                result.drive_dir = -1;
            }

            // 現在角度の近くにある目標角度を作る
            result.degree = currentAngleDeg + error;

            return result;
        }
};

TaskHandle_t control_loop_task_handle;

Motor              steering_motor_1(STEERING_MOTOR_DIR_1, STEERING_MOTOR_PWM_1, STEERING_MOTOR_CH_1);
IncrementalEncoder steering_encoder_1(STEERING_ENCODER_A_1, STEERING_ENCODER_B_1);
LimitSwitch        steering_limit_switch_1(STEERING_LIMIT_SW_1);
AnglePID           steering_pid_1(STEERING_PID_PARAM.p_gain, STEERING_PID_PARAM.i_gain, STEERING_PID_PARAM.d_gain,
                                  -STEER_MOTOR_POWER_LIMIT, STEER_MOTOR_POWER_LIMIT, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT, RANGE);
Steering steering_1(&steering_motor_1, &steering_encoder_1, &steering_limit_switch_1, &steering_pid_1, OFFSET_DEG_1, 1);

Motor              steering_motor_2(STEERING_MOTOR_DIR_2, STEERING_MOTOR_PWM_2, STEERING_MOTOR_CH_2);
IncrementalEncoder steering_encoder_2(STEERING_ENCODER_A_2, STEERING_ENCODER_B_2);
LimitSwitch        steering_limit_switch_2(STEERING_LIMIT_SW_2);
AnglePID           steering_pid_2(STEERING_PID_PARAM.p_gain, STEERING_PID_PARAM.i_gain, STEERING_PID_PARAM.d_gain,
                                  -STEER_MOTOR_POWER_LIMIT, STEER_MOTOR_POWER_LIMIT, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT, RANGE);
Steering steering_2(&steering_motor_2, &steering_encoder_2, &steering_limit_switch_2, &steering_pid_2, OFFSET_DEG_2, 2);

Motor              steering_motor_3(STEERING_MOTOR_DIR_3, STEERING_MOTOR_PWM_3, STEERING_MOTOR_CH_3);
IncrementalEncoder steering_encoder_3(STEERING_ENCODER_A_3, STEERING_ENCODER_B_3);
LimitSwitch        steering_limit_switch_3(STEERING_LIMIT_SW_3);
AnglePID           steering_pid_3(STEERING_PID_PARAM.p_gain, STEERING_PID_PARAM.i_gain, STEERING_PID_PARAM.d_gain,
                                  -STEER_MOTOR_POWER_LIMIT, STEER_MOTOR_POWER_LIMIT, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT, RANGE);
Steering steering_3(&steering_motor_3, &steering_encoder_3, &steering_limit_switch_3, &steering_pid_3, OFFSET_DEG_3, 3);

RobomasMotor drive_motor_1(DRIVE_MOTOR_ID_1);
RobomasMotor drive_motor_2(DRIVE_MOTOR_ID_2);
RobomasMotor drive_motor_3(DRIVE_MOTOR_ID_3);

target_vec_data target_data;
CAN             can(CAN_RX_PIN, CAN_TX_PIN);

PID drive_pid_1(DRIVE_PID_PARAM.p_gain, DRIVE_PID_PARAM.i_gain, DRIVE_PID_PARAM.d_gain, -DRIVE_MOTOR_POWER_LIMIT,
                DRIVE_MOTOR_POWER_LIMIT, -DRIVE_INTEGRAL_LIMIT, DRIVE_INTEGRAL_LIMIT);
PID drive_pid_2(DRIVE_PID_PARAM.p_gain, DRIVE_PID_PARAM.i_gain, DRIVE_PID_PARAM.d_gain, -DRIVE_MOTOR_POWER_LIMIT,
                DRIVE_MOTOR_POWER_LIMIT, -DRIVE_INTEGRAL_LIMIT, DRIVE_INTEGRAL_LIMIT);
PID drive_pid_3(DRIVE_PID_PARAM.p_gain, DRIVE_PID_PARAM.i_gain, DRIVE_PID_PARAM.d_gain, -DRIVE_MOTOR_POWER_LIMIT,
                DRIVE_MOTOR_POWER_LIMIT, -DRIVE_INTEGRAL_LIMIT, DRIVE_INTEGRAL_LIMIT);

// 後ろのはID printのときなどに使用
Drive drive_1(&drive_motor_1, &drive_pid_1, 1);
Drive drive_2(&drive_motor_2, &drive_pid_2, 2);
Drive drive_3(&drive_motor_3, &drive_pid_3, 3);

SwerveDrive swerve_drive_1(&drive_1, &steering_1, 1);
SwerveDrive swerve_drive_2(&drive_2, &steering_2, 2);
SwerveDrive swerve_drive_3(&drive_3, &steering_3, 3);

SwerveDrive*     swerve_drives[]    = {&swerve_drive_1, &swerve_drive_2, &swerve_drive_3};
constexpr size_t NUM_SWERVE_MODULES = 3;

struct CalibrationContext {
        // どの独ステ
        SwerveDrive* module;
        // どのイベントグループ
        EventGroupHandle_t events;
        // どの完了ビット
        EventBits_t done_bit;
        // 原点取り成功したかどうか
        bool success;
};

void calibration_task(void* parameter) {
    // calibrationContextに変換
    auto* context = static_cast<CalibrationContext*>(parameter);

    // 0点取り実行
    context->success = context->module->init();

    if (!context->success) {
        Serial.println("Calibration failed");
    }
    // 終わったら通知
    xEventGroupSetBits(context->events, context->done_bit);
    // タスク終了
    vTaskDelete(nullptr);
}

bool initialize_swerve_drives() {
    // FreeRTOSのイベントグループを作ってる。通知をまとめて管理できる。
    EventGroupHandle_t events = xEventGroupCreate();
    // イベントグループの作成失敗
    if (events == nullptr) {
        return false;
    }

    // CalibrationContextの作成
    CalibrationContext contexts[NUM_SWERVE_MODULES];
    // 作成に成功したタスクの完了ビット
    EventBits_t created_bits = 0;

    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        // Contextを設定
        contexts[i] = {
            swerve_drives[i],
            events,
            // 符号なし整数をビット操作　完了ビットを分けてる
            static_cast<EventBits_t>(1U << i),
            false,
        };

        // キャリブレーションタスクを作ってる。3つのキャリブレーションを並行に実行
        if (xTaskCreate(calibration_task, "CalibrationTask", 4096, &contexts[i], 10, nullptr) != pdPASS) {

            // ここまでに作成されたタスクがあれば、
            // それらの終了を待つ
            if (created_bits != 0) {
                xEventGroupWaitBits(events, created_bits, pdTRUE, pdTRUE, portMAX_DELAY);
            }
            vEventGroupDelete(events);
            return false;
        }

        // このタスクの作成に成功した
        created_bits |= static_cast<EventBits_t>(1U << i);
    }

    // すべてのモジュールが原点取りに成功したかを表す。1000 - 0001 = 0111
    const EventBits_t all_done = static_cast<EventBits_t>((1U << NUM_SWERVE_MODULES) - 1U);

    // 全モジュールのキャリブレーションが終わるまで待つ
    xEventGroupWaitBits(events, all_done, pdTRUE, pdTRUE, portMAX_DELAY);

    // 全モジュールの成功/失敗を確認
    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        if (!contexts[i].success) {
            vEventGroupDelete(events);
            return false;
        }
    }

    // EventGroupはもう不要
    vEventGroupDelete(events);

    return true;
}

void drive_pid_reset() {
    drive_pid_1.reset();
    drive_pid_2.reset();
    drive_pid_3.reset();
}

void stop_swerve_drives() {
    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        swerve_drives[i]->stop_drive();
    }
    drive_pid_reset();
}

struct ModulePosition {
        double x_mm;
        double y_mm;
};

// 実機の車輪位置に合わせて変更してください
// x: 前後方向、y: 左右方向
const ModulePosition MODULE_POSITIONS[NUM_SWERVE_MODULES] = {
    {0,        390.06 }, // module 1
    {-337.802, -195.03}, // module 2
    {337.802,  -195.03}, // module 3
};

void set_robot_velocity(double vx_mm_s, double vy_mm_s, double omega_deg_s) {

    // 小さい速度指令を0にする
    if (hypot(vx_mm_s, vy_mm_s) < TRANSLATION_DEADZONE_MM_S) {
        vx_mm_s = 0.0;
        vy_mm_s = 0.0;
    }

    if (fabs(omega_deg_s) < ROTATION_DEADZONE_DEG_S) {
        omega_deg_s = 0.0;
    }

    const double omega_rad_s = omega_deg_s * M_PI / 180.0;

    double        wheel_speed[NUM_SWERVE_MODULES];
    static double wheel_angle[NUM_SWERVE_MODULES];
    double        max_speed = 0.0;

    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        const double x = MODULE_POSITIONS[i].x_mm;
        const double y = MODULE_POSITIONS[i].y_mm;

        // 各車輪位置での速度ベクトル
        const double wheel_vx = vx_mm_s - omega_rad_s * y;
        const double wheel_vy = vy_mm_s + omega_rad_s * x;

        wheel_speed[i] = hypot(wheel_vx, wheel_vy);

        if (!(vx_mm_s == 0.0 && vy_mm_s == 0.0 && omega_deg_s == 0.0)) {
            wheel_angle[i] = atan2(wheel_vy, wheel_vx) * 180.0 / M_PI;
        }

        // 車輪で一番早いものを探す
        if (wheel_speed[i] > max_speed) {
            max_speed = wheel_speed[i];
        }
    }

    // 最大速度を超えないように全輪を同じ比率でスケーリング
    const double scale = max_speed > DRIVE_MAX_SPEED_MM_S ? DRIVE_MAX_SPEED_MM_S / max_speed : 1.0;

    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        const double speed = wheel_speed[i] * scale;

        swerve_drives[i]->set_target_mm_s(wheel_angle[i], speed);
    }
}

void handle_controller_input_deg_vec(int x_vec, int y_vec, uint8_t drive_power) {
    double magnitude = hypot((double)x_vec, (double)y_vec);

    if (magnitude <= MAGNITUDE_DEADZONE) {
        stop_swerve_drives();
        return;
    }

    double degree            = atan2((double)y_vec, (double)x_vec) * 180.0 / M_PI;
    double drive_target_duty = drive_power;

    for (size_t i = 0; i < NUM_SWERVE_MODULES; i++) {
        swerve_drives[i]->set_target_duty(degree, drive_target_duty);
    }
}

void handle_controller_input(int x_vec, int y_vec, uint8_t l2_value, uint8_t r2_value) {
    constexpr double MAX_TRANSLATION_SPEED_MM_S = DRIVE_MAX_SPEED_MM_S;
    constexpr double MAX_ROTATION_SPEED_DEG_S   = MAX_ROTATE_SPEED_DEG_S;

    constexpr double STICK_MAX   = 127.0;
    constexpr double TRIGGER_MAX = 255.0;

    // 右スティック：並進
    const double vx = static_cast<double>(x_vec) / STICK_MAX * MAX_TRANSLATION_SPEED_MM_S;
    const double vy = static_cast<double>(y_vec) / STICK_MAX * MAX_TRANSLATION_SPEED_MM_S;

    // R2 - L2：旋回
    // 符号が逆なら l2_value と r2_value を入れ替える
    const double omega =
        (static_cast<double>(l2_value) - static_cast<double>(r2_value)) / TRIGGER_MAX * MAX_ROTATION_SPEED_DEG_S;

    set_robot_velocity(vx, vy, omega);
}

void update_swerve_drives() {
    for (size_t i = 0; i < NUM_SWERVE_MODULES; i++) {
        swerve_drives[i]->update(CONTROL_CYCLE_S);
    }
}

void can_send() {
    // ドライブモータの指令値を CAN で送信
    if (!can.send(&drive_motor_1, &drive_motor_2, 0, &drive_motor_3)) {
        Serial.println("Drive CAN send failed");
    }
}

void control_loop_task(void* args) {
    TickType_t wake_time = xTaskGetTickCount();

    while (true) {
        update_swerve_drives();

        can_send();

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }
}

void setup() {
    Serial.begin(115200);
    if (!can.begin()) {
        Serial.println("CAN begin failed");
        while (true) {
        }
    }

    can.setOdometryData(&target_data);

    if (!initialize_swerve_drives()) {
        while (true) {
        }
    }
    PS4.begin("08:d1:f9:37:41:f2");
    xTaskCreate(control_loop_task, "ControlLoopTask", CONTROL_LOOP_TASK_STACK_SIZE, NULL, CONTROL_LOOP_TASK_PRIORITY,
                &control_loop_task_handle);
}

void loop() {
    // if (!PS4.isConnected()) {
    //     stop_swerve_drives();
    //     Serial.println("PS4 not connected");
    //     delay(100);
    //     return;
    // }

    // int     rx     = PS4.RStickX();
    // int     ry     = PS4.RStickY();
    // uint8_t r2_val = PS4.R2Value();
    // uint8_t l2_val = PS4.L2Value();

    // handle_controller_input_deg_vec(rx, ry, r2_val);
    // handle_controller_input(rx, ry, l2_val, r2_val);

    set_robot_velocity(target_data.x_mm_s, target_data.y_mm_s, target_data.theta_deg_s);
    // set_robot_velocity(1000, 0, 0);

    // static uint32_t last_print_time = 0;
    // const uint32_t  now             = millis();

    // if (now - last_print_time >= 1000) {
    //     last_print_time = now;
    //     Serial.printf("x:%d y:%d deg:%d receive:%d\r\n", target_data.x_mm_s, target_data.y_mm_s, target_data.theta_deg_s,
    //                   target_data.received);
    // }

    delay(LOOP_DELAY_MS);
}