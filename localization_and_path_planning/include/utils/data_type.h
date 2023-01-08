#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

namespace sensor{

typedef struct LaserScan_
{	
	LaserScan_()
	{

	}

	~LaserScan_()
	{

	}

	LaserScan_( const LaserScan_& rhs ) : time_stamp( rhs.time_stamp )
	{
		memcpy( angles, rhs.angles, 160 * sizeof( float ) );
		memcpy( dists, rhs.dists, 160 * sizeof( uint16_t ) );
	}

	const LaserScan_& operator=( const LaserScan_& rhs )
	{
		if( &rhs == this ){
			return *this;
		}

		time_stamp = rhs.time_stamp;
		memcpy( angles, rhs.angles, 160 * sizeof( float ) );
                memcpy( dists, rhs.dists, 160 * sizeof( uint16_t ) );

		return *this;
	}

	uint32_t time_stamp;
  	float angles[160];
  	uint16_t dists[160];
}LaserScan;

typedef struct _LidarPara
{
	float k0;
	float b0;
	float k1;
	float b1;
	float bias;
}LidarPara;

// ¿ØÖÆÁ¿
typedef struct Control_
{
	Control_()
	{

	}

	Control_( const float v_, const float w_ ) : v( v_ ), w( w_ )
	{

	}

	float v = 0;
	float w = 0;
}Control;

typedef struct RecordData_
{
	RecordData_(const float x_, const float y_, const float theta_, const sensor::LaserScan& scan_)
		: x(x_), y(y_), theta(theta_), scan_data(scan_)
	{

	}

	float x;
	float y;
	float theta;

	sensor::LaserScan scan_data;
}RecordData;

}

#endif
