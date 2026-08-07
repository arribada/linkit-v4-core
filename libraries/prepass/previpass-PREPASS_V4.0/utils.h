/*
 * utils.h
 *
 *  Created on: 11 déc. 2019
 *      Author: acombe
 */

#ifndef UTILS_H_
#define UTILS_H_


char su_date_jmahms_stu20 (	long	ev_jour    ,
                      		long	ev_mois    ,
                      		long	ev_annee   ,
                      		long	ev_heure   ,
                      		long	ev_minute  ,
                      		long	ev_seconde ,
                      		long	*dv_sec90  );


char su_date_stu20_jmahms  ( 	long	dv_sec90   ,
    	                	long 	*ev_jour    ,
                     		long	*ev_mois    ,
                     		long	*ev_annee   ,
                     		long	*ev_heure   ,
                     		long	*ev_minute  ,
                     		long	*ev_seconde );

float mod_angle(float angle);

float diff_angle(float a, float b);

void quicksort_index(long tableVal[], int tableIndex[], int NbElement);

// Use this function instead of '%' operator which is compiler dependant
long safe_modulo(long a, long m);

#endif /* UTILS_H_ */
