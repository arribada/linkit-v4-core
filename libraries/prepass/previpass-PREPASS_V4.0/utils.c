
#include <stdbool.h>
#include <math.h>

#include "prepas.h"
#include "utils.h"



int partition(long tableVal[], int index[], long low, long high);
void quicksort_index2(long tableVal[], int index[], long low, long high);


const long	ek_quanti[13][2] = {	{0,0},
	{31,31},
	{60,59},
	{91,90},
	{121,120},
	{152,151},
	{182,181},
	{213,212},
	{244,243},
	{274,273},
	{305,304},
	{335,334},
	{0,0}};

/* +-------------------------------------------------------------------+*/
/* |                                                                   |*/
/* | OPERATION : SU_DATE_JMAHMSM_STU90                                 |*/
/* |                                                                   |*/
/* +-------------------------------------------------------------------+*/
/* |                                                                   |*/
/* | Role : Date conversion (day,month,year,hour,minute,second) to     |*/
/* |        (seconds since 01/01/2020 00:00:00)                        |*/
/* |                                                                   |*/
/* |                                                                   |*/
/* | Input parameters                                                  |*/
/* | ----------------                                                  |*/
/* |    ev_jour   : Jour          (1<= jour <=31)                      |*/
/* |    ev_mois   : Mois          (1<= mois <= 12)                     |*/
/* |    ev_annee  : Annee         ( > 1900)                            |*/
/* |    ev_heure  : Heure         (0 <= heure <= 23)                   |*/
/* |    ev_minute : Minute        (0 <= minute <= 59)                  |*/
/* |    ev_seconde: Secondes      (0 <= seconde <= 59)                 |*/
/* |                                                                   |*/
/* | Donnees en sortie                                                 |*/
/* | -----------------                                                 |*/
/* |                                                                   |*/
/* |    dv_sec_90 : Nombre de secondes entieres ecoulees depuis le     |*/
/* |                01-Jan-2020 00:00:00                               |*/
/* |    jv_status : compte rendu d'execution                           |*/
/* |                                                                   |*/
/* +-------------------------------------------------------------------+*/

char su_date_jmahms_stu20 (	long	ev_jour    ,
		long	ev_mois    ,
		long	ev_annee   ,
		long	ev_heure   ,
		long	ev_minute  ,
		long	ev_seconde ,
		long	*dv_sec20  )
{

	/* +-------------------------------------------------------------------+*/
	/* +                  V A R I A B L E S   L O C A L E S                +*/
	/* +-------------------------------------------------------------------+*/


	long	jv_nb_annees; /* nombre d'annees ecoulees depuis 2020	*/
	long	jv_nb_annbis; /* nombre d'annees bissextiles 		*/
	long	jv_nb_jours;  /* nbre de jours ecoules depuis 2020 	*/
	long	jv_ind_bis;   /* indicateur annee courante 		*/

	/* +-------------------------------------------------------------------+*/
	/* +                            C O D E                                +*/
	/* +-------------------------------------------------------------------+*/

	/*    DEBUT SU_DATE_JMAHMSM_STU90	*/
	/*    ---------------------------	*/

	/* Test if 1 <= ev_mois <= 12	*/
	if ((ev_mois > 0) && (ev_mois < 13))
	{

		/*    CALCUL DU NOMBRE DE JOURS ECOULES DEPUIS 2020 ET LA FIN DE	*/
		/*    L'ANNEE PRECEDENTE						*/

		/*    Nombre d'annees ecoulees 	*/

		jv_nb_annees = ev_annee - 2020;

		/*    Nombre d'annees bissextiles depuis 2020	*/

		jv_nb_annbis = (ev_annee-1 -1900 ) / 4 - 29; //29 = nbre d'année bissextiles entre 1900 et 2020

		jv_nb_jours = (jv_nb_annees * 365) + jv_nb_annbis;

		/*    CALCUL DU NOMBRE DE JOURS ECOULES DEPUIS LE DEBUT DE L'ANNEE	*/

		/*    on regarde si l'annee courante est une annee bissextile		*/
		/*    jv_ind_bis = 1 annee bissextile					*/
		/*    jv_ind_bis = 2 annee non bissextile				*/

		jv_ind_bis = MIN((fmod(ev_annee,4)+1),2);

		jv_nb_jours = 	jv_nb_jours
			+ ek_quanti[ev_mois-1][jv_ind_bis-1]
			+ ev_jour
			- 1;

		/*    CALCUL DU NOMBRE DE SECONDES ECOULEES DEPUIS 2020			*/

		*dv_sec20 = jv_nb_jours * 86400 +
			ev_heure    * 3600  +
			ev_minute   * 60    +
			ev_seconde;


		return (0);
	}
	else
	{
		return (1);
	}

	/*    FIN SU_DATE_JMAHMSM_STU90	*/
}

/* +-------------------------------------------------------------------+*/
/* |                                                                   |*/
/* | OPERATION : SU_DATE_STU90_JMAHMS                                  |*/
/* |                                                                   |*/
/* +-------------------------------------------------------------------+*/
/* |                                                                   |*/
/* | Role : Date conversion  (seconds since 01/01/2020 00:00:00) to    |*/
/* |        (day,month,year,hour,minute,second)                        |*/
/* |                                                                   |*/
/* |                                                                   |*/
/* | Parametres en entree                                              |*/
/* | --------------------                                              |*/
/* |     dv_sec20 : Nombre de secondes ecoulees depuis le 01-Jan-2020  |*/
/* |                00:00:00                                           |*/
/* |                                                                   |*/
/* | Donnees en sortie                                                 |*/
/* | -----------------                                                 |*/
/* |    ev_jour   : Jour          (1<= jour <=31)                      |*/
/* |    ev_mois   : Mois          (1<= mois <= 12)                     |*/
/* |    ev_annee  : Annee         ( > 1900)                            |*/
/* |    ev_heure  : Heure         (0 <= heure <= 23)                   |*/
/* |    ev_minute : Minute        (0 <= minute <= 59)                  |*/
/* |    ev_seconde: Secondes      (0 <= seconde <= 59)                 |*/
/* |    jv_status : compte rendu d'execution                           |*/
/* |                                                                   |*/
/* +-------------------------------------------------------------------+*/


char su_date_stu20_jmahms  ( 	long	dv_sec20   ,
		long 	*ev_jour    ,
		long	*ev_mois    ,
		long	*ev_annee   ,
		long	*ev_heure   ,
		long	*ev_minute  ,
		long	*ev_seconde )
{

	/* +-------------------------------------------------------------------+*/
	/* +                  V A R I A B L E S   L O C A L E S                +*/
	/* +-------------------------------------------------------------------+*/

	long	jv_nb_jours;	/* nombre de jours ecoules 		*/
	long	jv_nb_annees;	/* nombre d'annees ecoulees 		*/

	long	ev_nb_annees_bis;/* nombre d'annees bissextiles 	*/
	long	ev_ind_bis;      /* indic. annee bissextile 		*/

	long	iv_trav;	/* variable de travail 			*/
	float	dv_trav;	/* variable de travail 			*/

	/* +-------------------------------------------------------------------+*/
	/* +                            C O D E                                +*/
	/* +-------------------------------------------------------------------+*/

	if (dv_sec20 >= 0)
	{

		/*    CALCUL DU NOMBRE DE JOURS ECOULEES DEPUIS 2020	*/

		jv_nb_jours = dv_sec20 / 86400;

		/*    CALCUL HEURE	*/

		iv_trav  = dv_sec20 - jv_nb_jours * 86400;
		*ev_heure = iv_trav / 3600;

		/*    CALCUL MINUTE	*/

		iv_trav   = iv_trav - *ev_heure * 3600;
		*ev_minute = iv_trav / 60;

		/*    CALCUL SECONDE	*/

		iv_trav    = iv_trav - *ev_minute * 60;
		*ev_seconde = iv_trav;

		/*    AJUSTEMENT NB_JOURS HEURE MINUTE SECONDE SI LE NOMBRE	*/
		/*    DE MICROSECONDES CALCULEES EST >= 1.D6			*/

		if (*ev_seconde >= 60)
		{
			*ev_seconde = *ev_seconde - 60;
			*ev_minute  = *ev_minute + 1;
			if (*ev_minute >= 60)
			{
				*ev_minute = *ev_minute - 60;
				*ev_heure  = *ev_heure + 1;
				if (*ev_heure >= 24)
				{
					*ev_heure = *ev_heure - 24;
					jv_nb_jours = jv_nb_jours + 1;
				}
			}
		}

		/*    CALCUL NOMBRE D'ANNEES ECOULEES DEPUIS 2020	*/

		dv_trav      = (float) (jv_nb_jours);
		dv_trav      = (dv_trav + 0.5) / 365.25;
		jv_nb_annees = (int) (dv_trav);

		/*    CALCUL ANNEE */

		*ev_annee = jv_nb_annees + 2020;

		/*    CALCUL DU NOMBRE D'ANNEES BISSEXTILES PASSEES DEPUIS 2020	*/

		ev_nb_annees_bis =  (*ev_annee-1 -1900) / 4 - 29;

		/*    Test si annee courante bissextile		*/
		/*    ind_bis = 1 -> annee courante bissextile	*/
		/*    ind_bis = 2 -> sinon			*/

		ev_ind_bis = MIN((fmod(*ev_annee,4)+1),2);

		/*    CALCUL DU NOMBRE DE JOURS ECOULES DANS L'ANNEE	*/

		jv_nb_jours = 	jv_nb_jours
			- (jv_nb_annees * 365)
			- ev_nb_annees_bis
			+ 1;

		/*    CALCUL NO DU MOIS DANS L'ANNEE*/

		*ev_mois = 1;
		while (
				(jv_nb_jours > ek_quanti[*ev_mois-1][ev_ind_bis-1]) &&
				(*ev_mois <= 12))
		{
			*ev_mois = *ev_mois + 1;
		}
		/* END WHILE	*/

		*ev_mois = *ev_mois - 1;

		/*    CALCUL DU NUMERO DU JOURS DANS LE MOIS	*/

		*ev_jour = jv_nb_jours -  ek_quanti[*ev_mois-1][ev_ind_bis-1];

		return (0);
	}
	else
	{
		return (1);
	}

	/*    FIN SU_DATE_STU90_JMAHMSM	*/
	/*    -------------------------	*/

}



float mod_angle(float x) {
	float y;
	y = fmod(x, 360);
	if (x < 0) {
		y += 360;
	}
	return y;
}


float diff_angle(float a, float b) {
	float y = mod_angle(b - a);
	if (y > 180) {
		y -= 360;
	}
	return y;
}


int partition(long tableVal[], int tableIndex[], long low, long high)
{

	int tmp, i, j;
	long pivot;

	pivot = tableVal[tableIndex[high]];
	i = (low - 1);

	for (j = low; j < high; j++)
	{
		if (tableVal[tableIndex[j]] < pivot)
		{
			i++;
			tmp = tableIndex[i];
			tableIndex[i] = tableIndex[j];
			tableIndex[j] = tmp;
		}
	}

	tmp = tableIndex[i+1];
	tableIndex[i+1] = tableIndex[high];
	tableIndex[high] = tmp;

	j = i + 1;

	return j;
}


void quicksort_index2(long tableVal[], int tableIndex[], long low, long high)
{

	int pi;

	if (low < high)
	{
		pi = partition(tableVal, tableIndex, low, high);
		quicksort_index2(tableVal, tableIndex, low, pi - 1);
		quicksort_index2(tableVal, tableIndex, pi + 1, high);
	}

}


void quicksort_index(long tableVal[], int tableIndex[], int NbElement)
{
	int i;
	for (i = 0; i < NbElement; ++i) tableIndex[i] = i;
	quicksort_index2(tableVal, tableIndex, 0, NbElement - 1);
}

// Use this function instead of '%' operator which is compiler dependant
long safe_modulo(long a, long m) {
	long r = a % m;
	return (r < 0) ? r + m : r;
}
