
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "utils.h"
#include "prepas.h"



/* +-------------------------------------------------------------------+*/
/* +                      variables globales                           +*/
/* +-------------------------------------------------------------------+*/

int num_sat;
st_aop * aop;
float S_LAT_B;
float C_LAT_B;
float S_LON_B;
float C_LON_B;
float Re2;
float longitude_balise;
float latitude_balise;
st_satvars SATVARS[NB_MAX_SAT];

#define EARTH_RADIUS 6378137.0        // meters (WGS84 equatorial radius)
#define EARTH_POLAR_RADIUS 6356752.3  // meters (WGS84 polar radius)

/* +-------------------------------------------------------------------+*/
/* +                      C O N S T A N T E S                          +*/
/* +-------------------------------------------------------------------+*/



float calcul_longitude_sat(float cumpso);
float eval_elev(float cumpso);
float eval_cos_do(float cumpso);
float eval_do(float cumpso);
float compute_azimuth(float cumpso);
void coord_geodesic_to_cartesian(
		float lon_deg,
		float lat_deg,
		float h,
		float device_position[3]);


int prepas (st_config * config, st_aop * ptr_aop, int Nsat, st_res * tab_resus, int * nb_visi)
{

	int N_INTERVAL, i, num_previsi, num_period;
	int num_visi, nb_previsi, num_iter, num_interval;

	float dpso_deg_sec, cumpso_calcul_start, cumpso_calcul_end, nodal_per;
	float b, c, d, D, cos_dlon;
	float DLON_MAX, lat_sat_max, lat_sat_inf, lat_sat_sup;
	float pso_deb1, pso_deb2, pso_fin1, pso_fin2, pso_deb, pso_fin;
	float lon_sat_inf, lon_sat_sup, delta_pso, dpso_start_deb;
	float do_deb, slope_deb, do_fin, slope_fin;
	float d2pso_deg_sec2, orb_per, nb_orb, orb_per_ini;
	float cumpso_deb, cumpso_fin, cumpso_deb_ini, cumpso_fin_ini, dpso_deb, dpso_fin;
	float lon_deb, lon_fin, lon_min, lon_max, lon_min2, lon_max2;
	float cumpso_elevmax, pass_elev_max, pass_duration, delta_t, time_margin_min;
	float lon_min_F, dlon_min_max, DO_margin, loop_period, duration;

	long date_debut_sec20, date_fin_sec20, DT_start_sec, DT_stop_sec, T_START, T_STOP;
	long t_visi, t_visi_ini, t_start_visi, t_end_visi, lon_min_I, dt_stop;

	bool STOP, flag_all_lon, no_final_date, flag_exit;

	st_delta_t dt;
	st_res tmp;
	st_loopstate * LS;

	st_pre_visi table_pre_visi[NB_MAX_PREVISI];
	int sortIdx[NB_MAX_PREVISI];
	long dates_pre_visi[NB_MAX_PREVISI];

	const float eps_conv_do = 0.1;
	const int max_iter = 10;


	longitude_balise = config->pf_lon;
	latitude_balise = config->pf_lat;
	S_LAT_B = sind(config->pf_lat);
	C_LAT_B = cosd(config->pf_lat);
	S_LON_B = sind(config->pf_lon);
	C_LON_B = cosd(config->pf_lon);
	Re2 = RT * RT;


	/**
	 * Marge � prendre sur DO_MAX
	 * prise en compte de l'excentricit� des satellites (exc=0.001) : 0.2�
	 * prise en compte de l'applatissement terrestre : 0.4� * sin(LAT_BALISE)
	 * prise en compte de l'erreur de positionnement de la balise : erreur / 111km;
	 */
	DO_margin = 0.2 + 0.4 * S_LAT_B + config->marge_position / 111.0;



	// Conversion des dates calendaires en secondes �coul�es depuis le 01/01/2020
	su_date_jmahms_stu20(config->jour_deb, config->mois_deb, config->an_deb,
			config->heure_deb, config->min_deb, config->sec_deb, &date_debut_sec20);


	if (config->an_fin == 0 || config->mois_fin == 0 || config->jour_fin == 0)
	{
		no_final_date = true;
		date_fin_sec20 = 0;

	} else {
		no_final_date = false;
		su_date_jmahms_stu20(config->jour_fin, config->mois_fin, config->an_fin,
				config->heure_fin, config->min_fin, config->sec_fin, &date_fin_sec20);
	}



	/*
	 * ETAPE 1: Pr�-calculs
	 * --------------------------------------------------------------------------------------
	 */



	for (num_sat = 0; num_sat < Nsat; num_sat++) {

		aop = &(ptr_aop[num_sat]);
		SATVARS[num_sat].SI = sind(aop->inc);
		SATVARS[num_sat].CI = cosd(aop->inc);
		SATVARS[num_sat].dga2 = aop->dga * aop->dga;
		SATVARS[num_sat].D_elev_neg = sqrt(SATVARS[num_sat].dga2 - Re2);

		nodal_per = aop->ts * 60; //p�riode nodale en secondes
		su_date_jmahms_stu20(aop->jour_bul, aop->mois_bul, aop->an_bul, aop->heure_bul, aop->min_bul, aop->sec_bul, &(SATVARS[num_sat].date_aop_sec20));
		dpso_deg_sec = 360.0 / nodal_per; // vitesse de variation de la pso en deg/sec

		DT_start_sec = date_debut_sec20 - SATVARS[num_sat].date_aop_sec20;
		DT_stop_sec = date_fin_sec20 - SATVARS[num_sat].date_aop_sec20;
		cumpso_calcul_start = dpso_deg_sec * (float)DT_start_sec;
		cumpso_calcul_end = dpso_deg_sec * (float)DT_stop_sec;

		/*
		 * Parties enti�re et flottante de la variation de longitude
		 */
		SATVARS[num_sat].d_noeud_I = (long)floor(aop->d_noeud);
		SATVARS[num_sat].d_noeud_F = aop->d_noeud - (float)SATVARS[num_sat].d_noeud_I;


		if (aop->dgap < 0) {
			SATVARS[num_sat].T_DOT = 3.0 * PI * aop->dgap / 86400.0 * sqrt(aop->dga * 1000 / EARTH_MU); //[sec/sec]
			SATVARS[num_sat].LNA_DOT_DOT = -360 * SATVARS[num_sat].T_DOT * nodal_per / (2 * 86400); // [deg/orbite�]

			/* Calcul de l'acc�l�ration de variation de la pso en deg/sec/sec (approximation de Taylor) */
			d2pso_deg_sec2 = -1.5 * dpso_deg_sec * aop->dgap / (86400.0 * aop->dga * 1000);
			cumpso_calcul_start += 0.5 * d2pso_deg_sec2 * (float)DT_start_sec * (float)DT_start_sec;
			cumpso_calcul_end += 0.5 * d2pso_deg_sec2 * (float)DT_stop_sec * (float)DT_stop_sec;

		} else {
			SATVARS[num_sat].T_DOT = 0;
			SATVARS[num_sat].LNA_DOT_DOT = 0;
		}

		/* calcul de la p�riode orbitale initiale */
		orb_per_ini = nodal_per + SATVARS[num_sat].T_DOT * DT_start_sec;

		/* Calcul de la distance orthodromique max pour avoir une visi */
		b = 2.0 * RT * sind(config->elevation_min);
		c = Re2 - SATVARS[num_sat].dga2;
		d = b * b - 4.0 * c;
		D = (-b + sqrt(d)) / 2.0;
		SATVARS[num_sat].DO_MAX = DO_margin + asind(D * cosd(config->elevation_min) / aop->dga);

		/* Calcul de l'�cart en longitude max pour avoir une visi */
		cos_dlon = (cosd(SATVARS[num_sat].DO_MAX) - S_LAT_B * S_LAT_B) / (C_LAT_B * C_LAT_B);
		cos_dlon = MAX(-1.0, cos_dlon);
		cos_dlon = MIN(1.0, cos_dlon);
		DLON_MAX = acosd(cos_dlon);


		/*
		 * Recherche des PSO pour lesquelles LAT_SAT est compris dans l'intervalle [lat_balise-DO_MAX; lat_balise-DO_MAX+DO_MAX] (1 ou 2 possibilit�s)
		 * LAT_SAT est strictement croissante entre les pso [0;90] et [270;360], strictement d�croissante entre [90;270]
		 */

		lat_sat_max = aop->inc;
		if (lat_sat_max > 90.0) {
			lat_sat_max = 180.0 - lat_sat_max;
		}

		lat_sat_inf = config->pf_lat - SATVARS[num_sat].DO_MAX;
		lat_sat_sup = config->pf_lat + SATVARS[num_sat].DO_MAX;

		pso_deb2 = 0.0;
		pso_fin2 = 0.0;

		if (lat_sat_sup > lat_sat_max) {
			/* 1 solution, p�le nord */
			pso_deb1 = asind(sind(lat_sat_inf) / SATVARS[num_sat].SI);
			pso_fin1 = mod_angle(180.0 - pso_deb1);
			N_INTERVAL = 1;

		} else if (lat_sat_inf < -lat_sat_max) {
			/* 1 solution, p�le sud */
			pso_fin1 = asind(sind(lat_sat_sup) / SATVARS[num_sat].SI) + 360.0;
			pso_deb1 = mod_angle(180.0 - pso_fin1);
			N_INTERVAL = 1;

		} else {
			/* 2 solutions */
			pso_deb1 = mod_angle(asind(sind(lat_sat_inf) / SATVARS[num_sat].SI));
			pso_fin1 = mod_angle(asind(sind(lat_sat_sup) / SATVARS[num_sat].SI));
			pso_deb2 = mod_angle(180.0 - pso_fin1);
			pso_fin2 = mod_angle(180.0 - pso_deb1);

			N_INTERVAL = 2;
		}



		/* recherche des longitudes sat min et max pour ces intervalles de PSO */
		lon_sat_inf = mod_angle(config->pf_lon - DLON_MAX);
		lon_sat_sup = mod_angle(config->pf_lon + DLON_MAX);

		flag_all_lon = (DLON_MAX > (180.0 - FLOAT_EPS))? 1: 0;


		SATVARS[num_sat].N_INTERVAL = N_INTERVAL;


		for (i = 1; i <= N_INTERVAL; i++)
		{

			if (i == 1) {
				pso_deb = pso_deb1;
				pso_fin = pso_fin1;
			} else {
				pso_deb = pso_deb2;
				pso_fin = pso_fin2;
			}

			delta_pso = diff_angle(pso_deb, pso_fin);
			dpso_start_deb = mod_angle(pso_deb - cumpso_calcul_start);


			if (config->include_current_visi)
			{
				/* d�calage avec la pso de d�part */
				float shift_pso, shift_T;
				shift_pso = diff_angle(pso_deb, cumpso_calcul_start);
				if (shift_pso > 0.0 && diff_angle(pso_fin, cumpso_calcul_start) < 0.0) {
					/* d�calage de cumpso_calcul_start � pso_deb */
					cumpso_calcul_start -= shift_pso;
					dpso_start_deb = 0.0;
					shift_T = shift_pso / 360 * orb_per_ini;
					DT_start_sec -= (long)shift_T;
				}
			}


			cumpso_deb = cumpso_calcul_start + dpso_start_deb;
			cumpso_fin = cumpso_deb + delta_pso;
			lon_deb = calcul_longitude_sat(cumpso_deb);
			lon_fin = calcul_longitude_sat(cumpso_fin);


			if (aop->inc >= 90.0)
			{
				/* orbite r�trograde : la longitude est d�croissante */
				lon_min = lon_fin;
				lon_max = lon_deb;
			} else {
				/* la longitude est croissante */
				lon_min = lon_deb;
				lon_max = lon_fin;
			}

			/* On ne garde que le delta lon_min/lon_max, et on ne manipule que lon_min par la suite */
			dlon_min_max = lon_max - lon_min;

			dt.jj = floor(DT_start_sec / 86400.0);
			dt.sec = (float)safe_modulo(DT_start_sec, 86400) + dpso_start_deb / 360.0 * orb_per_ini;
			if (dt.sec > 86400.0) {
				dt.jj++;
				dt.sec -= 86400.0;
			}


			/*
			 * Dans l'algorithme qui suit:
			 * lon_min ne prend pas en compte la r�gression du DGA
			 * lon_min2 la prend en compte
			 */

			lon_min2 = lon_min;
			if (aop->dgap < 0.0) {
				//retranche � lon_min la correction driftDGA qui a �t� prise en compte dans calcul_longitude_sat
				nb_orb = cumpso_deb / 360.0;
				lon_min -= SATVARS[num_sat].LNA_DOT_DOT * nb_orb * nb_orb;
			}

			/*
			 * D�composition de lon_min en parties enti�res et flottantes. Cette �tape est n�cessaire pour les probl�mes de pr�cision
			 * machine quand on somme lon_min+aop->d_noeud un grand nombre de fois
			 */
			lon_min_I = (long)floor(lon_min);
			lon_min_F = lon_min - lon_min_I;

			lon_max2 = lon_min2 + dlon_min_max;

			//Sauvegarde �tat initial de la boucle
			LS = &(SATVARS[num_sat].loopstate[i - 1]);
			LS->lon_sat_inf = lon_sat_inf;
			LS->lon_sat_sup = lon_sat_sup;
			LS->cumpso_fin = cumpso_fin;
			LS->cumpso_deb = cumpso_deb;
			LS->dt.jj = dt.jj;
			LS->dt.sec = dt.sec;
			LS->lon_min_I = lon_min_I;
			LS->lon_min_F = lon_min_F;
			LS->dlon_min_max = dlon_min_max;
			LS->lon_max2 = lon_max2;
			LS->lon_min2 = lon_min2;
			LS->flag_all_lon = flag_all_lon;
			LS->nb_orb = nb_orb;
		}
	}




	/*
	 * Optimisation de la p�riode de bouclage
	 */

	loop_period = LOOP_PERIOD_J;
	if (config->Npass > 0 && config->Npass <= 10) {
		/* Peu de visi demand�es : on passe � 1 orbite de bouclage */
		loop_period = 0.0694444;
	} else if (!no_final_date) {
		/* Dur�e demand�e inf�rieure � 0.5j : on traite en une seule it�ration */
		duration = ((float)(date_fin_sec20 - date_debut_sec20)) / 86400.0;
		if (loop_period > duration) loop_period = duration;
	}

	//    printf("\nloop_period:%f\n", loop_period);



	num_period = 1;
	STOP = false;
	num_visi = 0;
	*nb_visi = 0;

	while (!STOP)
	{

		T_START = (num_period == 1)? date_debut_sec20: T_STOP;
		T_STOP = T_START + (long)(loop_period * 86400.0);
		printf("\nnext period\n");
		printf("T_START: %d\n", T_START);
		printf("T_STOP: %d\n", T_STOP);
		printf("date_fin_sec20: %d\n", date_fin_sec20);
		printf("no_final_date: %d\n", no_final_date);

		/*
		 * ETAPE 2.1: Calculs des cr�neaux possibles de visibilit�s
		 * --------------------------------------------------------------------------------------
		 */

		num_previsi = 0;

		for (num_sat = 0; num_sat < Nsat; num_sat++) {

			aop = &(ptr_aop[num_sat]);
			dt_stop = T_STOP - SATVARS[num_sat].date_aop_sec20;

			for (num_interval = 0; num_interval < SATVARS[num_sat].N_INTERVAL; num_interval++)
			{

				LS = &(SATVARS[num_sat].loopstate[num_interval]);

				while (LS->dt.jj * 86400L + (long)LS->dt.sec <= dt_stop)
				{

					/* D�passement de capacit� du tableau. Cela ne devrait pas arriver si NB_MAX_PREVISI est correctement dimensionn� */
					if (num_previsi == NB_MAX_PREVISI) break;

					if (LS->flag_all_lon ||
							(diff_angle(LS->lon_sat_inf, LS->lon_min2) > 0 && diff_angle(LS->lon_sat_sup, LS->lon_min2) < 0 ) ||
							(diff_angle(LS->lon_sat_inf, LS->lon_max2) > 0 && diff_angle(LS->lon_sat_sup, LS->lon_max2) < 0 ) ||
							(diff_angle(LS->lon_sat_inf, LS->lon_min2) < 0 && diff_angle(LS->lon_sat_sup, LS->lon_max2) > 0 ) ||
							(diff_angle(LS->lon_sat_inf, LS->lon_min2) > 0 && diff_angle(LS->lon_sat_sup, LS->lon_max2) < 0 ))
					{
						t_visi = SATVARS[num_sat].date_aop_sec20 + LS->dt.jj * 86400L + (long)LS->dt.sec;

						/* La date est stock�e uniquement pour le tri chrono des pre-visi */
						dates_pre_visi[num_previsi] = t_visi;
						table_pre_visi[num_previsi].num_sat = num_sat;
						table_pre_visi[num_previsi].cumpso_deb = LS->cumpso_deb;
						table_pre_visi[num_previsi].cumpso_fin = LS->cumpso_fin;
						//printf("previsi %d  %d  %f  %f\n", t_visi, num_sat, LS->cumpso_deb, LS->cumpso_fin);

						num_previsi++;
					}

					/* passage � l'orbite suivante */
					LS->cumpso_fin += 360.0;
					LS->cumpso_deb += 360.0;

					/* recalcul de la p�riode orbitale et de la dur�e �coul�e */
					orb_per = aop->ts * 60;
					if (aop->dgap < 0) {
						orb_per += SATVARS[num_sat].T_DOT * (LS->dt.jj * 86400.0 + LS->dt.sec);
					}

					LS->dt.sec += orb_per;
					if (LS->dt.sec > 86400.0) {
						LS->dt.jj++;
						LS->dt.sec -= 86400.0;
					}

					/* Somme lon_min += d_noeud */
					LS->lon_min_I += SATVARS[num_sat].d_noeud_I;
					LS->lon_min_F += SATVARS[num_sat].d_noeud_F;

					LS->lon_min2 = (float)(LS->lon_min_I) + LS->lon_min_F;
					if (aop->dgap < 0) {
						LS->nb_orb += 1.0;
						LS->lon_min2 += SATVARS[num_sat].LNA_DOT_DOT * LS->nb_orb * LS->nb_orb;
					}

					LS->lon_max2 = LS->lon_min2 + LS->dlon_min_max;
				}
			}
		}

		nb_previsi = num_previsi;


		/* tri chronologique des PRE-VISI */
		if (nb_previsi > 0) {
			quicksort_index(dates_pre_visi, sortIdx, nb_previsi);
		}


		/*
		 * ETAPE 2.2: Calculs des caract�ristiques pr�cises des visibilit�s
		 * --------------------------------------------------------------------------------------
		 */

		for (num_previsi = 0; num_previsi < nb_previsi; num_previsi++)
		{
			t_visi_ini = dates_pre_visi[sortIdx[num_previsi]];
			num_sat = table_pre_visi[sortIdx[num_previsi]].num_sat;
			cumpso_deb_ini = table_pre_visi[sortIdx[num_previsi]].cumpso_deb;
			cumpso_fin_ini = table_pre_visi[sortIdx[num_previsi]].cumpso_fin;

			aop = &(ptr_aop[num_sat]);

			cumpso_deb = cumpso_deb_ini;
			cumpso_fin = cumpso_fin_ini;

			/* approximation affine de la DO au d�part de la courbe, pour avoir la cumpso de l'�l�vation min */
			do_deb = eval_do(cumpso_deb);
			slope_deb = eval_do(cumpso_deb + 1.0) - do_deb;

			do_fin = eval_do(cumpso_fin);
			slope_fin = eval_do(cumpso_fin + 1.0) - do_fin;

			if (slope_deb > 0 || slope_fin < 0) {
				/* courbe de DO d�croissante au d�part, ou croissante � la fin, il n'y aura pas de visi sur ce passage */
				continue;
			}

			flag_exit = false;

			num_iter = 1;
			while ((fabs(do_deb - SATVARS[num_sat].DO_MAX) > eps_conv_do) && (num_iter < max_iter))
			{
				if (num_iter > 1) {
					/* l'�valuation de la pente et le test de la valeur sont d�j� effectu�s pour la �re it�ration */
					slope_deb = eval_do(cumpso_deb + 1.0) - do_deb;
					if (slope_deb > 0) {
						/* on est pass� de l'autre c�t� du sommet, le passage a une �l�vation min inf�rieure */
						flag_exit = true;
						break;
					}
				}
				cumpso_deb = (SATVARS[num_sat].DO_MAX - do_deb + slope_deb * cumpso_deb) / slope_deb;
				do_deb = eval_do(cumpso_deb);
				num_iter++;
			}
			if (flag_exit) continue;

			num_iter = 1;
			while ((fabs(do_fin - SATVARS[num_sat].DO_MAX) > eps_conv_do) && (num_iter < max_iter))
			{
				if (num_iter > 1) {
					slope_fin = eval_do(cumpso_fin + 1.0) - do_fin;
					if (slope_fin < 0) {
						/* on est pass� de l'autre c�t� du sommet, le passage a une �l�vation min inf�rieure */
						flag_exit = true;
						break;
					}
				}
				cumpso_fin = (SATVARS[num_sat].DO_MAX - do_fin + slope_fin * cumpso_fin) / slope_fin;
				do_fin = eval_do(cumpso_fin);
				num_iter++;
			}
			if (flag_exit) continue;

			if (cumpso_deb >= cumpso_fin)
			{
				/* le sommet de la courbe d'�l�vation est < config.elevation_min */
				continue;
			}


			/* Calcul de l'�l�vation max (= �l�vation en milieu de passage) */
			cumpso_elevmax = (cumpso_deb + cumpso_fin) / 2.0;
			pass_elev_max = eval_elev(cumpso_elevmax);


			/*  Filtrages sur l'�l�vation */
			if (pass_elev_max < config->elevation_min || pass_elev_max > config->max_elevation_max || pass_elev_max < config->min_elevation_max)
			{
				continue;
			}

			dpso_deb = cumpso_deb - cumpso_deb_ini;
			dpso_fin = cumpso_fin - cumpso_deb_ini;
			orb_per = aop->ts * 60.0 + SATVARS[num_sat].T_DOT * (float)(t_visi_ini - SATVARS[num_sat].date_aop_sec20);
			t_start_visi = t_visi_ini + (long)(dpso_deb * orb_per / 360.0);
			t_end_visi = t_visi_ini + (long)(dpso_fin * orb_per / 360.0);
			pass_duration = ((float)(t_end_visi - t_start_visi)) / 60.0;


			/*  Filtrage sur la dur�e de passage */
			if (pass_duration < config->duree_min)
			{
				continue;
			}


			/* Ajout de la marge temporelle sur la date de d�but et la dur�e du passage */
			if (config->marge_temporelle > 0)
			{
				delta_t = ((float)(t_start_visi - SATVARS[num_sat].date_aop_sec20)) / 86400.0;
				time_margin_min = config->marge_temporelle * delta_t / SIX_MONTH;
				t_start_visi -= (long)(time_margin_min * 60.0);
				pass_duration += 2.0 * time_margin_min;
			}

			/* Filtrages sur les dates */
			if (num_period == 1 && config->include_current_visi)
			{
				/* On peut avoir d�tect� une visi qui finit avant m�me la date de d�but du calcul, on les �carte ici */
				long t_stop_visi;
				if (t_start_visi < date_debut_sec20) {
					t_stop_visi = t_start_visi + (long)(pass_duration * 60);
					if (t_stop_visi < date_debut_sec20) {
						continue;
					}
				}
			}

			/* On peut avoir une visi qui est apr�s la date de fin demand�e */
			if (!no_final_date && t_start_visi > date_fin_sec20) {
				continue;
			}


			/* Stockage du passage */
			tab_resus[num_visi].num_sat = num_sat;
			tab_resus[num_visi].delta_start = t_start_visi - date_debut_sec20;
			tab_resus[num_visi].pass_elev_max = pass_elev_max;
			tab_resus[num_visi].pass_duration = pass_duration;
			//printf("visi  %d  %d  %f  %f\n", num_sat, t_start_visi - date_debut_sec20, pass_elev_max, pass_duration);

			/* Compute azimuth angles at three dates : start, middle and end of satellite pass */
			tab_resus[num_visi].start_azimuth = compute_azimuth(cumpso_deb);
			tab_resus[num_visi].middle_azimuth = compute_azimuth(cumpso_elevmax);
			tab_resus[num_visi].end_azimuth = compute_azimuth(cumpso_fin);

			num_visi++;
			*nb_visi = num_visi;

			/*
			 * On arr�te dans ces 2 cas :
			 * - Le nombre de visibilit� a atteint la capacit� du tableau
			 * - Le nombre de visibilit� a atteint le nombre demand�
			 */
			if (*nb_visi == NB_MAX_VISI || (config->Npass != 0 && *nb_visi == config->Npass)) {
				STOP = true;
				break;
			}
		}

		/* On arr�te dans ce cas: La date de fin demand�e est d�pass�e */
		if (!no_final_date && T_STOP >= date_fin_sec20) {
			STOP = true;
		}

		num_period++;
	}


	//    printf("nb_period:%d\n", num_period - 1);


	/*
	 * Les visibilit�s sont class�es par date de d�but du cr�neau de possible visibilit�. La date ajust�e peut donc amener
	 * 2 visibilit�s a �tre invers�e chronologiquement
	 */

	for (num_visi = 1; num_visi < *nb_visi; ++num_visi) {
		if (tab_resus[num_visi].delta_start < tab_resus[num_visi-1].delta_start) {
			/* swap */
			tmp.num_sat = tab_resus[num_visi-1].num_sat;
			tmp.delta_start = tab_resus[num_visi-1].delta_start;
			tmp.pass_elev_max = tab_resus[num_visi-1].pass_elev_max;
			tmp.pass_duration = tab_resus[num_visi-1].pass_duration;

			tab_resus[num_visi-1].num_sat = tab_resus[num_visi].num_sat;
			tab_resus[num_visi-1].delta_start = tab_resus[num_visi].delta_start;
			tab_resus[num_visi-1].pass_elev_max = tab_resus[num_visi].pass_elev_max;
			tab_resus[num_visi-1].pass_duration = tab_resus[num_visi].pass_duration;

			tab_resus[num_visi].num_sat = tmp.num_sat;
			tab_resus[num_visi].delta_start = tmp.delta_start;
			tab_resus[num_visi].pass_elev_max = tmp.pass_elev_max;
			tab_resus[num_visi].pass_duration = tmp.pass_duration;
		}
	}

	return 0;
}




float calcul_longitude_sat(float cumpso)
{
	float lna, lon_sat, nb_orb;

	nb_orb = cumpso / 360.0;
	lna = aop->lon_asc + aop->d_noeud * nb_orb;
	if (aop->dgap < 0) {
		lna += SATVARS[num_sat].LNA_DOT_DOT * nb_orb * nb_orb;
	}

	lon_sat = atand(sind(cumpso) * SATVARS[num_sat].CI, cosd(cumpso)) + lna;
	lon_sat = mod_angle(lon_sat);

	return lon_sat;
}



float eval_cos_do(float cumpso) {
	float lat_sat, lon_sat, cos_DO;
	lat_sat = asind(sind(cumpso) * SATVARS[num_sat].SI);
	lon_sat = calcul_longitude_sat(cumpso);
	cos_DO = sind(lat_sat) * S_LAT_B + cosd(lat_sat) * C_LAT_B * cosd(longitude_balise - lon_sat);
	return cos_DO;
}

float eval_do(float cumpso) {
	return acosd(eval_cos_do(cumpso));
}


float eval_elev(float cumpso)
{
	float cos_DO, sin_DO, D, cos_elev, elev;
	cos_DO = eval_cos_do(cumpso);
	sin_DO = sqrt(1.0 - cos_DO * cos_DO);
	D = sqrt(Re2 + SATVARS[num_sat].dga2 - 2.0 * RT * aop->dga * cos_DO);
	cos_elev = aop->dga * sin_DO / D;
	elev = (cos_elev >= 1.0)? 0.0: acosd(cos_elev);
	if (D > SATVARS[num_sat].D_elev_neg) elev = -elev;
	return elev;
}


float compute_azimuth(float cumpso)
{
	// Algorithm from ESA GNSS Handbook Vol. 1, Annex B.3
	// https://gssc.esa.int/navipedia/GNSS_Book/ESA_GNSS-Book_TM-23_Vol_I.pdf

	float device_position[3];
	float satellite_position[3];
	float line_of_sight[3];
	float lat_sat, lon_sat, alt_sat, norm, x, y, azimuth_rad, azimuth_deg;

	coord_geodesic_to_cartesian(longitude_balise, latitude_balise, 0.0, device_position);

	lat_sat = asind(sind(cumpso) * SATVARS[num_sat].SI);
	lon_sat = calcul_longitude_sat(cumpso);
	alt_sat = aop->dga * 1000.0 - EARTH_RADIUS;
	coord_geodesic_to_cartesian(lon_sat, lat_sat, alt_sat, satellite_position);

	line_of_sight[0] = satellite_position[0] - device_position[0];
	line_of_sight[1] = satellite_position[1] - device_position[1];
	line_of_sight[2] = satellite_position[2] - device_position[2];
	norm = sqrt(line_of_sight[0] * line_of_sight[0] + line_of_sight[1] * line_of_sight[1] + line_of_sight[2] * line_of_sight[2]);
	line_of_sight[0] /= norm;
	line_of_sight[1] /= norm;
	line_of_sight[2] /= norm;

	y = line_of_sight[1] * C_LON_B - line_of_sight[0] * S_LON_B;
	x = line_of_sight[2] * C_LAT_B - line_of_sight[0] * C_LON_B * S_LAT_B - line_of_sight[1] * S_LON_B * S_LAT_B;
	azimuth_rad = atan2(y, x);
	azimuth_deg = mod_angle(azimuth_rad * 180.0 / PI);

	return azimuth_deg;
}


void coord_geodesic_to_cartesian(float lon_deg, float lat_deg, float altitude, float device_position[3])
{
	float GE, GP, lat_param, cos_lat_param;

	GE = EARTH_RADIUS + altitude;
	GP = EARTH_POLAR_RADIUS + altitude;

	lat_param = atan((GP * tand(lat_deg)) / GE);
	if (lat_param >= PI / 2.0) {
		lat_param = PI / 2.0;
	} else if (lat_param <= -PI / 2.0) {
		lat_param = -PI / 2.0;
	}

	cos_lat_param = cos(lat_param);

	device_position[0] = GE * cos_lat_param * cosd(lon_deg);
	device_position[1] = GE * cos_lat_param * sind(lon_deg);
	device_position[2] = GP * sin(lat_param);
}
