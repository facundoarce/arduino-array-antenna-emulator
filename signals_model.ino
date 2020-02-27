/*********************************************************************************/
/********* MODEL OF SIGNALS RECEIVED BY THE ELEMENTS OF THE LINEAL ARRAY *********/
/*********************************************************************************/

/* CONSTANTS */
#define c       3e8         // Propagation speed in free space [m/s]
#define K       4           // Number of radiators/elements in array
#define Re      6371.0      // Radius of the Earth [km]
#define RePOW2  Re * Re     // Re^2
#define Me      5.98e24     // Mass of the Earth [kg]
#define G       6.673e-11   // Gravitational constant [0.001 * N*m^2/kg^2]

/* USER DEFINED VARIABLES */
double h = 650.0;           // Flying altitude [km]
uint32_t f0 = 150e6;        // Frequency of radiation [Hz]
double d_elem = 1.0;        // Distance between radiators/elements [m]

/* DERIVED VARIABLES */
double lambda0;         // Wavelenght [m]
double k0;              // Free space wave number [deg/m]
double r;               // Radius of the satellite trajectory [km]
double rPOW2;           // r^2
double v_t;             // Tangential velocity of satellite [km/s]
double w;               // Angular velocity [rad/s]
double alpha_max;       // Angle of a pass with respect to Earth [rad]
double t_end;           // Time of pass [s]

void derive_variables ( void )
{
  lambda0 = c / f0;
  k0 = 360 / lambda0;
  r = Re + h;
  rPOW2 = pow( r, 2 );
  v_t = sqrt( ( double ) G * Me / ( r / 1000 ) ) * 1e-6;
  w = v_t / r;
  alpha_max = 2.0 * acos( Re / r );
  t_end = alpha_max / w;

  Serial.print( "INITIALIZING PARAMETERS\r" );
  Serial.print( String( (String)"lambda0 = " + lambda0 + "\r" ) );
  Serial.print( String( (String)"k0 = " + k0 + "\r" ) );
  Serial.print( String( (String)"r = " + r + "\r" ) );
  Serial.print( String( (String)"v_t = " + v_t + "\r" ) );
  Serial.print( String( (String)"w = " + w + "\r" ) );
  Serial.print( String( (String)"alpha_max = " + alpha_max + "\r" ) );
  Serial.print( String( (String)"t_end = " + t_end + "\r" ) );
  Serial.print( String( (String)"Re = " + Re + "\r" ) );
  Serial.print( String( (String)"rPOW2 = " + rPOW2 + "\r" ) );
  Serial.print( String( (String)"RePOW2 = " + RePOW2 + "\r\r" ) );
}

double distance_satellite_antenna_t ( double t )
{
  return sqrt( rPOW2 + RePOW2 - 2 * r * Re * cos( alpha_max / 2.0 - w * t ) );
}

double distance_satellite_antenna_theta ( double theta )
{
  
  return sqrt( rPOW2 - RePOW2 * pow( sin( theta ), 2 ) ) - Re * cos( theta );
}

double angle_relative_to_broadside ( double t, double d )
{
  double x = r * sin( alpha_max / 2 - w * t ) / d;
  if ( x < -1.0 ) x = -1.0;
  else if ( x > 1.0 ) x = 1.0;
  return asin( x );
}

uint32_t frequency( double theta )
{
  return f0 + ( int32_t )( v_t * 1000.0 * sin( theta ) / c * f0 );
}

double phase( uint8_t elem, double theta )
{
  if ( elem <= K ) return 180.0 * ( K - 1 ) + k0 * ( K - elem ) * d_elem * sin( theta );
  else return -1;
}

double amplitude( double d )
{
  return h / d;
}
