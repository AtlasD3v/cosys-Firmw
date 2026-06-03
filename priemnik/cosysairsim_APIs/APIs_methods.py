import cosysairsim as airsim
import time
class Cosys_client:
    DRONE_NAME = "MyDrone"
    AIRSIM_IP = "127.0.0.1"

    def __init__(self):
        self.client = None
        self.initialize_client()

    def initialize_client(self):
        self.client = airsim.MultirotorClient(self.AIRSIM_IP)
        self.client.reset()
        self.client.confirmConnection()
        print("[BRIDGE] Подключились к Cosys-AirSim")
        time.sleep(2)

    # def get_imu(self):
    #     imu = self.client.getMultirotorState(vehicle_name=self.DRONE_NAME) #заменить на getIMU
    #     imu_data_array = [imu.kinematics_estimated.angular_velocity.x_val, imu.kinematics_estimated.angular_velocity.y_val, imu.kinematics_estimated.angular_velocity.z_val, imu.kinematics_estimated.linear_acceleration.x_val, imu.kinematics_estimated.linear_acceleration.y_val, imu.kinematics_estimated.linear_acceleration.z_val]
    #     return imu_data_array
    def get_imu(self):
        imu = self.client.getImuData(imu_name='MyImu', vehicle_name=self.DRONE_NAME)
        imu_data_array = [imu.angular_velocity.x_val, imu.angular_velocity.y_val, imu.angular_velocity.z_val, imu.linear_acceleration.x_val, imu.linear_acceleration.y_val, imu.linear_acceleration.z_val ]

        return imu_data_array
    
    def moveByMotorPWMsAsync(self, front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration):
        self.client.moveByMotorPWMsAsync(
            front_right_pwm=front_right_pwm,
            rear_left_pwm=rear_left_pwm, 
            front_left_pwm=front_left_pwm, 
            rear_right_pwm=rear_right_pwm,
            duration= duration, 
            vehicle_name=self.DRONE_NAME
        )
    
    
    def enable_airsim_api(self):
        self.client.enableApiControl(is_enabled=True, vehicle_name=self.DRONE_NAME) #self.DRONE_NAME равен имени дрона, которое прописывается в файле settings.json
        print("+++++ ВКЛЮЧИЛИ API +++++")

    def arm_drone(self):
        print("****[BRIDGE] АРМИРУЕМ ДРОН****")
        self.client.armDisarm(arm=True, vehicle_name=self.DRONE_NAME)   

    def get_gps_data(self):
        gps_data = self.client.getGpsData(gps_name="MyGPS", vehicle_name=self.DRONE_NAME)

        velocity_x = gps_data.gnss.velocity.x_val
        velocity_y = gps_data.gnss.velocity.y_val
        velocity_z = gps_data.gnss.velocity.z_val

        pos_latitude = gps_data.gnss.geo_point.latitude
        pos_longitude = gps_data.gnss.geo_point.longitude
        pos_altitude = gps_data.gnss.geo_point.altitude


        return [pos_latitude, pos_longitude, pos_altitude, velocity_x, velocity_y, velocity_z]
    
    # def get_velocity_from_gps_data(self):
    #     gps_data = self.get_gps_data()
    #     velocity_x = gps_data.gnss.velocity.x_val
    #     velocity_y = gps_data.gnss.velocity.y_val
    #     velocity_z = gps_data.gnss.velocity.z_val

    #     return [velocity_x, velocity_y, velocity_z]
    
    # def get_position_from_gps_data(self):
    #     gps_data = self.get_gps_data()
    #     pos_latitude = gps_data.gnss.geo_point.latitude
    #     pos_longitude = gps_data.gnss.geo_point.longitude
    #     pos_altitude = gps_data.gnss.geo_point.altitude

    #     return [pos_latitude, pos_longitude, pos_altitude]
    