ref struct SInitParam
{
	int IMUgrade;
	int GPStech;
	double scale;
	double azimuth;
	double xOffset;
	double yOffset;
	double latOrigin;
	double longOrigin;
public:
	SInitParam()
	{
		IMUgrade=1;
		GPStech=1;
		scale=1;
		azimuth=0;
		xOffset=0;
		yOffset=0;
		latOrigin=0;
		longOrigin=0;
	}
};