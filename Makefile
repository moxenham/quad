CC = g++
NAVIO = ../../Navio
INCLUDES = -I ../..

all:
	$(CC) $(INCLUDES) Quad.cpp mmapGpio.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/PCA9685.cpp $(NAVIO)/I2Cdev.cpp $(NAVIO)/Ublox.cpp $(NAVIO)/MS5611.cpp -o Quad -lrt -lpthread

clean:
	rm Quad
