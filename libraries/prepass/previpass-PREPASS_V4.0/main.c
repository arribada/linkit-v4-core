#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "prepas.h"
#include "utils.h"
#include <time.h>


//#define EMBEDDED



#ifdef EMBEDDED
#define PRINTF(fmt, ...) sprintf(buffer, (fmt), __VA_ARGS__);printf("\n%s\n",buffer);
#else
#define PRINTF(fmt, ...) sprintf(buffer, (fmt), __VA_ARGS__);printf("\n%s\n",buffer);
#endif


int main (int argc, char *argv[])
{

	int Nsat;     /* number of satellites in orbit file  */
	int status;
	int nb_visi;

#ifdef EMBEDDED

	Nsat = 8;
	st_aop tab_aop[NB_MAX_SAT] = {
		{"A1", 2021, 6, 7, 23, 9,  29,  6890.912,  97.4647,  98.723,  -23.751,  95.0032,   -3.52},
		{"MA", 2021, 6, 7, 22, 56,  18,  7195.532,  98.4708,  317.970,  -25.341,  101.3585,   0.00},
		{"MB", 2021, 6, 7, 22, 30,  15,  7195.615,  98.7166,  345.033,  -25.340,  101.3596,   0.00},
		{"MC", 2021, 6, 7, 23, 24,  3,  7195.594,  98.6865,  331.901,  -25.340,  101.3593,   0.00},
		{"15", 2021, 6, 7, 23, 8,  48,  7180.342,  98.6805,  303.809,  -25.259,  101.0377,   -0.10},
		{"18", 2021, 6, 7, 22, 54,  54,  7225.705,  98.9953,  343.500,  -25.497,  101.9943,   -0.91},
		{"19", 2021, 6, 7, 22, 0,  34,  7226.202,  99.1817,  312.724,  -25.498,  102.0043,   -0.70},
		{"SR", 2021, 6, 7, 22, 59,  8,  7160.169,  98.5429,  103.805,  -25.153,  100.6132,   -0.28}};


	st_res tab_resus[NB_MAX_VISI];

#define pf_lat 43.549
#define pf_lon 1.4848
#define an_deb 2021
#define mois_deb 6
#define jour_deb 22
#define heure_deb 0
#define min_deb 0
#define sec_deb 0
#define an_fin 2021
#define mois_fin 6
#define jour_fin 23
#define heure_fin 0
#define min_fin 0
#define sec_fin 0
#define elevation_min 5.0
#define max_elevation_max 90.0
#define min_elevation_max 0.0
#define duree_min 0.0
#define marge_temporelle 5.0
#define marge_position 0.0
#define Npass 0
#define include_current_visi 1

	st_config config = {pf_lon, pf_lat, an_deb, mois_deb, jour_deb, heure_deb, min_deb,
		sec_deb, an_fin, mois_fin, jour_fin, heure_fin, min_fin, sec_fin, elevation_min,
		max_elevation_max, min_elevation_max, duree_min, marge_temporelle, marge_position, Npass, include_current_visi};


	status = prepas(&config, tab_aop, Nsat, tab_resus, &nb_visi);
	printf("\n\nnb_visi:%d\n", nb_visi);


#else


	int i, isat;
	clock_t T1, T2;
	char currentdir[300];
	FILE *ul_conf;    /* configuration file */
	FILE *ul_bull;    /* orbit file   */
	FILE *ul_resu;    /* result file   */
	st_config config;              /* array of configuration parameter */
	st_aop tab_aop[NB_MAX_SAT];              /* array of orbit parameter  */
	st_res tab_resus[NB_MAX_VISI];
	char buffer[512];


	if (argc >= 2) {
		chdir(argv[1]);
	}
	getcwd(currentdir, 300);
	// sprintf(buffer, "Dossier courant : %s\n", currentdir);printf(buffer);
	PRINTF("Current directory : %s\n", currentdir)


	char nf_conf [] = "prepas_config.txt";
	char nf_bull [] = "aop_list.txt";
	char nf_resu [] = "prepas_results.txt";

	char ligne [MAXLU];
	long ev_jour, ev_mois, ev_annee, ev_heure, ev_minute, ev_seconde;
	char *token;

	printf("Starting program\n");


	/*-------- Read Data Configuration ---------*/

	/* Open configuration file */

	printf("\nReading configuration file\n");

	ul_conf = fopen(nf_conf, "r");
	if (ul_conf == NULL) {
		printf("Error while opening configuration file");
		return -1;
	}


	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%f", &(config.pf_lat));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%f", &(config.pf_lon));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%4d %02d %02d %02d %02d %02d ",
			&(config.an_deb),  &(config.mois_deb),
			&(config.jour_deb),  &(config.heure_deb),
			&(config.min_deb),  &(config.sec_deb));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%4d %02d %02d %02d %02d %02d ",
			&(config.an_fin),  &(config.mois_fin),
			&(config.jour_fin),  &(config.heure_fin),
			&(config.min_fin),  &(config.sec_fin));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%f", &(config.elevation_min));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%f", &(config.max_elevation_max));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%f", &(config.min_elevation_max));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%f", &(config.duree_min));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%f", &(config.marge_temporelle));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%f", &(config.marge_position));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%d", &(config.Npass));

	fgets (ligne, MAXLU, ul_conf);
	printf("%s",ligne);
	sscanf(ligne, "%d", (int*)&(config.include_current_visi));

	fclose (ul_conf);

	/* Close configuration file */



	/*-------- Read Orbit Parameter ---------*/

	/* Open Orbit parameter file */

	printf("\nReading satellite AOPs file\n");

	ul_bull = fopen(nf_bull, "r");
	if (ul_bull == NULL) {
		printf("Error while reading AOP file");
		return -1;
	}

	isat = 0;



	while (!feof(ul_bull)) {
		if (!fgets (ligne, MAXLU, ul_bull)) {
			//! Unable to read
			break;
		}
		int lineSize = strlen(ligne);

		if (lineSize == MAXLU - 1) {
			//! Overflow
			break;
		}
		if (lineSize < 10) {
			//! Empty line
			continue;
		}
		printf("%s",ligne);

		token = strtok(ligne, " ");
		if (token == NULL) break;

		strcpy(tab_aop[isat].sat, token);

		for (i = 0; i < 5; ++i) token = strtok(NULL, " ");
		sscanf(token, "%d", &tab_aop[isat].an_bul);

		token = strtok(NULL, " ");
		sscanf(token, "%d", &tab_aop[isat].mois_bul);

		token = strtok(NULL, " ");
		sscanf(token, "%d", &tab_aop[isat].jour_bul);

		token = strtok(NULL, " ");
		sscanf(token, "%d", &tab_aop[isat].heure_bul);

		token = strtok(NULL, " ");
		sscanf(token, "%d", &tab_aop[isat].min_bul);

		token = strtok(NULL, " ");
		sscanf(token, "%d", &tab_aop[isat].sec_bul);

		token = strtok(NULL, " ");
		sscanf(token, "%f", &tab_aop[isat].dga);

		token = strtok(NULL, " ");
		sscanf(token, "%f", &tab_aop[isat].inc);

		token = strtok(NULL, " ");
		sscanf(token, "%f", &tab_aop[isat].lon_asc);

		token = strtok(NULL, " ");
		sscanf(token, "%f", &tab_aop[isat].d_noeud);

		token = strtok(NULL, " ");
		sscanf(token, "%f", &tab_aop[isat].ts);

		token = strtok(NULL, " ");
		sscanf(token, "%f", &tab_aop[isat].dgap);

		isat++;
		if (isat >= NB_MAX_SAT) {
			printf("Number of AOP > NB_MAX_SAT, ignoring rest of AOPs...");
			break;
		}
	}
	Nsat = isat;
	printf("\n\nNsat: %d\n", Nsat);
	printf("Latitude: %f\n", config.pf_lat);
	printf("Longitude: %f\n", config.pf_lon);
	printf("Start date: %d %d %d %d %d %d\n", config.an_deb, config.mois_deb, config.jour_deb, config.heure_deb, config.min_deb, config.sec_deb);
	printf("End date: %d %d %d %d %d %d\n", config.an_fin, config.mois_fin, config.jour_fin, config.heure_fin, config.min_fin, config.sec_fin);

	fclose (ul_bull);


	T1 = clock();
	for (i = 0; i < 1; ++i)
		status = prepas(&config, tab_aop, Nsat, tab_resus, &nb_visi);
	T2 = clock();

	printf("\n\nTotal number of found visibilities: %d\n", nb_visi);
	printf("\n\nExecution duration: %f\n", (double)(T2 - T1) / 1000);


	if (status != 0) {
		printf("Error while executing prepas. return code = %d\n", status);

	} else {

		long date_debut_sec20;

		ul_resu = fopen(nf_resu, "w");

		fprintf(ul_resu, "sat;dt_start_j;pass_elev_max;pass_duration;start_azimuth;middle_azimuth;end_azimuth\n");

		su_date_jmahms_stu20(config.jour_deb, config.mois_deb, config.an_deb, config.heure_deb, config.min_deb, config.sec_deb, &date_debut_sec20);

		for (i = 0; i < nb_visi; ++i) {
			su_date_stu20_jmahms(tab_resus[i].delta_start + date_debut_sec20, &ev_jour, &ev_mois, &ev_annee, &ev_heure, &ev_minute, &ev_seconde);
			fprintf(ul_resu, "%s;%4ld-%02ld-%02ld %02ld:%02ld:%02ld;%6.2f;%6.2f;%9.2f;%9.2f;%9.2f\n", tab_aop[tab_resus[i].num_sat].sat, ev_annee, ev_mois, ev_jour, ev_heure, ev_minute, ev_seconde,
					tab_resus[i].pass_elev_max, tab_resus[i].pass_duration, tab_resus[i].start_azimuth, tab_resus[i].middle_azimuth, tab_resus[i].end_azimuth);
		}

		fclose(ul_resu);
	}

#endif



	return 0;
}



