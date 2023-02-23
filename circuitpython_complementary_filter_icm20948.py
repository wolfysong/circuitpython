# libraries
import time
import math
import board
from adafruit_icm20x import ICM20948

# here we define a class for filtering IMU data
class complimentary_filter:
    # critical data
    acc = [0., 0., 0.]
    gyro = [0., 0., 0.]

    #time information
    time_prev = 0
    time_now = 0.
    dt = 0.001

    #3 types of angles
    angle_filtered = [0., 0.]
    angle_gyro = [0., 0.]
    angle_acc = [0., 0.]

    #weight of the complementary filter
    weight = 0.98

    # private function for calculate acc anagle
    # check https://seanboe.me/blog/complementary-filters
    # check https://www.hobbytronics.co.uk/accelerometer-info
    def calculate_angle_acc(self):
        try:
            self.angle_acc[0] = math.atan(-self.acc[0] / math.sqrt(math.pow(self.acc[1], 2) + math.pow(self.acc[2], 2)));
            self.angle_acc[1] = math.atan(-self.acc[1] / math.sqrt(math.pow(self.acc[0], 2) + math.pow(self.acc[2], 2)));
        except:
            pass # WE USE TRY, EXCEPT TO PREVENT divided by zero

    #private function for calculate gyro anagle
    def calculate_angle_gyro(self):
        self.angle_gyro[0] = self.angle_gyro[0] + self.gyro[0]*self.dt;
        self.angle_gyro[1] = self.angle_gyro[1] + self.gyro[1]*self.dt;

    #complementary filter
    def calculate_angle_filter(self):
        self.calculate_angle_acc();
        self.angle_filtered[0] = self.weight*(self.angle_filtered[0] + self.gyro[1]*self.dt) + (1.- self.weight) * self.angle_acc[0];
        self.angle_filtered[1] = self.weight*(self.angle_filtered[1] + self.gyro[1]*self.dt) + (1.- self.weight) * self.angle_acc[1];

    #private function update data
    def update_data(self, acc = [0.,0.,0.], gyro = [0.,0.,0.]):
        self.acc = acc
        self.gyro = gyro
        self.time_now = time.monotonic()
        self.dt = self.time_now-self.time_prev
        self.time_prev = self.time_now

    #init the class
    def __init__(self, acc = [0.,0.,0.], gyro = [0.,0.,0.]):
        self.update_data(acc,gyro)

    #updating all functions
    def update_filter(self, acc = [0.,0.,0.], gyro = [0.,0.,0.]):
        self.update_data(acc,gyro)     # update data
        self.calculate_angle_gyro();   # not needed, for demo only
        self.calculate_angle_filter(); # filter

        # remember, all angles are in radian
        return self.angle_acc, self.angle_gyro, self.angle_filtered

#constant
rad_to_deg = 180./math.pi

i2c = board.I2C() #get i2c port
icm = ICM20948(i2c) # get the instance of the sensor on the i2c port

cf = complimentary_filter(); # define a filter

while True:

    time.sleep(0.01)

    #get filtered data
    angle_acc, angle_gyro, angle_filtered = cf.update_filter(list(icm.acceleration),list(icm.gyro))
    #output is radians
    output_data_list = angle_acc+angle_gyro+angle_filtered
    #to degrees
    output_data = [i * rad_to_deg for i in output_data_list]
    print(tuple(output_data))


