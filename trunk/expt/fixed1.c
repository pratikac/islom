#include <stdio.h>
#include <inttypes.h>


int main()
{
	//position is 24.8
	int32_t pos_fx=0;					//cm
	int32_t time_fx = 0;
	//velocity is 8.8
	int16_t vel_fx = 0;				//cm/s
	//accel is 8.8
	int16_t accel_fx = 32;				//32/256 cm/s/s
	//dt = 8.8
	int16_t dt_fx = 5;					//5/256 s
	
	float pos=0, vel=0, accel = 32/256.0, dt=5/256.0, time=0;
	while(1)				
	{
		time_fx += (int32_t)dt_fx;
		vel_fx += (int16_t)(accel_fx*dt_fx);
		if(vel_fx > 100*256)
			vel_fx = 100*256;
		pos_fx += vel_fx*dt_fx;

		time += dt;
		vel += accel*dt;
		if(vel > 125)
			vel = 125;
		pos += vel*dt;

		printf("t_fx: %f pos_fx: %f vel_fx: %f\n", (time_fx>>8) + ( (uint8_t)time_fx/256.0), (pos_fx>>8) + ( (uint8_t)pos_fx/256.0),(vel_fx>>8) + ( (uint8_t)vel_fx/256.0)   );

		printf("t: %f pos: %f vel: %f\n", time, pos, vel);
		printf("\n");

	}
	return 0;

}
