

#ifndef PREPAS_H_
#define PREPAS_H_


#define MIN(A,B)  ((A)<(B)?(A):(B))
#define MAX(A,B)  ((A)>(B)?(A):(B))

#define PI            3.141592653589793116
#define RAD2DEG       (180.0/PI)
#define DEG2RAD       (PI/180.0)
#define sind(A)       sin((A)*DEG2RAD)
#define asind(A)      asin(A)*RAD2DEG
#define cosd(A)       cos((A)*DEG2RAD)
#define acosd(A)      acos(A)*RAD2DEG
#define tand(A)       tan((A)*DEG2RAD)
#define atand(A, B)   atan2(A, B)*RAD2DEG

#define FLOAT_EPS 1.2e-7 //précision des floats 32 bits
#define RT 6378.137 //rayon terre en km
#define SIX_MONTH (6.0*30.0)
#define EARTH_MU 3.986004415e14

#define NB_MAX_VISI 50000
#define NB_MAX_SAT 30

#define LOOP_PERIOD_J 0.5
#define NB_PERIOD_PER_DAY 2 // (1/LOOP_PERIOD_J)
#define NB_MAX_ORB_PER_DAY 15
#define LOOP_PERIOD_ORB ((int)(NB_MAX_ORB_PER_DAY / NB_PERIOD_PER_DAY) + 2) //((int)(NB_MAX_ORB_PER_DAY * LOOP_PERIOD_J) + 2)
#define NB_MAX_PREVISI ((int)(NB_MAX_SAT * LOOP_PERIOD_ORB))


#define MAXLU 132


//typedef int bool;
#define false 0
#define true 1

typedef struct
{
	float pf_lon;
	float pf_lat;
	int an_deb;
	int mois_deb;
	int jour_deb;
	int heure_deb;
	int min_deb;
	int sec_deb;
	int an_fin;
	int mois_fin;
	int jour_fin;
	int heure_fin;
	int min_fin;
	int sec_fin;
	float elevation_min;
	float max_elevation_max;
	float min_elevation_max;
	float duree_min;
	float marge_temporelle;
	float marge_position;
	int Npass;
	bool include_current_visi;
} st_config;


typedef struct
{
	char sat[3];
	int an_bul; /* bulletin epoch */
	int mois_bul; /* bulletin epoch */
	int jour_bul; /* bulletin epoch */
	int heure_bul; /* bulletin epoch */
	int min_bul; /* bulletin epoch */
	int sec_bul; /* bulletin epoch */
	float dga; /* semi-major axis (km) */
	float inc; /* orbit inclination (deg) */
	float lon_asc; /* longitude of ascending node (deg) */
	float d_noeud; /* asc. node drift during one revolution (deg) */
	float ts; /* orbit period (min) */
	float dgap; /* drift of semi-major axis (m/jour) */
} st_aop;



typedef struct
{
	int num_sat; //numéro du satellite dans le tableau des AOP
	float delta_start; // secondes écoulées depuis le début du calcul
	float pass_elev_max; // élévation max en degrés
	float pass_duration; // durée du passage en minutes
	float start_azimuth; // azimuth at start of pass
	float middle_azimuth; // azimuth at middle of pass (max elevation)
	float end_azimuth; // azimuth at end of pass
} st_res;



typedef struct
{
	int num_sat;
	float cumpso_deb;
	float cumpso_fin;
} st_pre_visi;



/*
 * Date stockée avec un entier + un float pour augmenter la précision
 */
typedef struct
{
	float sec;
	long jj;
} st_delta_t;



/*
 * Structure contenant un état de parcours de boucle
 */
typedef struct
{
	float lon_sat_inf;
	float lon_sat_sup;
	float cumpso_fin;
	float cumpso_deb;
	st_delta_t dt;
	int lon_min_I;
	float lon_min_F;
	float dlon_min_max;
	float lon_min2;
	float lon_max2;
	bool flag_all_lon;
	float nb_orb;
} st_loopstate;



/*
 * Structure de données utilisées pour stocker des calculs concernant un sat, pour éviter le recalcul dans la seconde boucle
 */
typedef struct
{
	float SI;
	float CI;
	float dga2;
	float T_DOT;
	float LNA_DOT_DOT;
	float D_elev_neg;
	float DO_MAX;
	long date_aop_sec20;
	int d_noeud_I;
	float d_noeud_F;
	int N_INTERVAL;
	st_loopstate loopstate[2];
} st_satvars;





int prepas (st_config * params, st_aop * ptr_aop, int Nsat, st_res * tab_resus, int * nb_visi);


#endif /* PREPAS_H_ */
