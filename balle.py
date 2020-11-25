
import numpy as np
import random as rd
from jeu import *
from joueurs import *
import math
from PyQt5 import  QtGui, QtCore, QtWidgets, uic
import time


class balle():
    """
    La classe balle caracterise le mouvement de la balle au cours du jeu.

    Attributs :
    	rayon (int) : rayon de la balle
    	vi (float) : vitesse initiale de la balle
    	pos_ini (tuple) : position initiale de la balle

    Methodes :
    	actualisation (dt, frottement) : actualise la vitesse et la position de la balle
    	rebondmur (normale) : actualise la vitesse de la balle lors d'un impact avec un mur
    	rebondjoueur (no_joueur, joueur, force_joueur, frottement) : actualise la vitesse de la balle lors d'un impact avec un joueur
    	detectimpact (joueurtot) : detecte un impact avec un joueur selon la position de la balle
    	detectimpactmur (terrain) : detecte un impact avec un mur selon la position de la balle

    @author : P. Pineau T. Malledan
    """

    # ==================================================================================================================
    def __init__(self,rayon,vi,pos_ini):
        """
        Constructeur de la classe balle.

        Parametres :
        	rayon (int) : rayon de la balle
        	vi (float) : vitesse initiale de la balle
        	pos_ini (tuple) : position initiale de la balle
        """

        self.coord = pos_ini
        #On lance la balle avec un angle aleatoire entre -2 / 2 degre autour de l'axe y au debut de la partie
        # theta = np.deg2rad(rd.randrange(-3,3))
        # if theta == 0:
        #     theta = 1
        theta = rd.random()*np.pi/10 + np.pi/2
        if theta < (np.pi / 2) - 0.01 or theta > (np.pi / 2) + 0.01:
            theta = rd.random() * np.pi / 10 + np.pi / 2
        self.vitesse = (vi * np.cos(theta), vi * np.sin(theta))

        self.rayon = rayon
        self.contact = False
        self.jeu = True

    # @property
    # def vitesse(self):
    #     return self.__vitesse
    # @vitesse.setter
    # def vitesse(self,V):
    #     vx,vy = V
    #     if vx > 80:
    #         vx = 80
    #     if vy > 80 :
    #         vy = 80
    #     return (vx,vy)

    @property
    def position(self):
        return self.__position
    @position.setter
    def position(self, coord):
        x, y = coord
        if x < 20 :
            x = 20
        if x > 1480 :
            x = 1480
        if y < 20 :
            y = 20
        if y > 500 :
            y = 500
        return


    # ==================================================================================================================
    def actualisation(self,dt,frottement = 0.999995):
        """
        Methode qui actualise la vitesse et la position de la balle.

        Parametres :
        	dt (float) : temps d'iteration du programme
        	frottement (float, optionnel) : frottements appliques sur la balle (air, terrain)
        """

        x,y = self.coord
        vx,vy  = self.vitesse
        V = np.sqrt(vx ** 2 + vy ** 2)
        # V -= dt * frottement * V
        vx *= frottement
        vy *= frottement
        # if vx > 40 :
        #     vx = 40
        # if vy > 40:
        #     vy = 40

        self.coord = (x + vx * dt, y + vy * dt)
        if vx == 0 :
            self.vitesse = (0, vy )
        else :
            theta = np.arctan(vy / vx)
            self.vitesse = (vx, vy)

    # ==================================================================================================================
    def rebondmur(self, normale):

        """
        Methode qui actualise la vitesse de la balle lors d'un impact avec un mur.

        Parametre :
        	normale (int) : prend la valeur 0 si le mur est vertical, et 1 s'il est horizontal
        """

        vx,vy = self.vitesse
        if normale == 0 :
            self.vitesse = (-vx, vy)
        if normale == 1:
            self.vitesse = (vx, -vy)



    # ==================================================================================================================
    def rebondjoueur(self, no_joueur,joueur,force_joueur = 1.00001 , frottement = 1):

        """
        Methode qui actualise la vitesse de la balle lors d'un impact avec un mur.

        Parametre :
        	normale (int) : prend la valeur 0 si le mur est vertical, et 1 s'il est horizontal
        """

        x, y = self.coord
        vx, vy = self.vitesse
        V = (frottement * np.sqrt(vx ** 2 + vy ** 2)) * force_joueur
        vx = frottement * vx * force_joueur
        vy = frottement * vy * force_joueur
        RayonPied = joueur.rayon_pieds
        xpied = joueur.abs_joueurs
        ypied = joueur.ordo_j[no_joueur]
        RayonBalle = self.rayon

        xballe = x
        yballe = y
        ximpact = (xballe * RayonPied + xpied * RayonBalle)/(RayonPied + RayonBalle)
        yimpact = (ypied * RayonBalle + yballe * RayonPied)/(RayonPied + RayonBalle)

        #calcul de theta
        x1 = ximpact - xballe #vecteur directeur balle
        y1 = yimpact - yballe

        x2 = xpied - ximpact #vecteur directeur normale
        y2 = ypied - yimpact

        dot = x1 * x2 + y1 * y2  # dot product
        det = x1 * y2 - y1 * x2  # determinant
        theta = math.atan2(det, dot)  # angle entre la trajectoire de la balle et la normale a l'impact

        #calcul alpha , equation normale : y = ax + b
        a = (ypied - yimpact)/(xpied - ximpact)
        b = ypied - a * xpied
        alpha = np.arctan((-a * ypied) / (b + a * xpied))

        #clacul de la vitesse dans le repere local
        vxR0 =  -(vx*np.cos(alpha) - vy * np.sin(alpha))
        # vyR0 =  V * np.sin(theta)
        vyR0 =(vy * np.cos(alpha) + vx * np.sin(alpha))

        # calcul de la vitesse dans le repere global
        vxrebond = vyR0 * np.sin(alpha) + vxR0 * np.cos(alpha)
        vyrebond = vyR0 * np.cos(alpha) - vxR0 * np.sin(alpha)

        self.vitesse = (vxrebond, vyrebond)

    # ==================================================================================================================
    def detectimpact(self,joueurtot):
        """
        Methode qui detecte un impact avec un joueur selon la position de la balle.

        Parametre :
        	joueurtot (liste) : liste de la forme [[gardien1, defense1, demi1, attaque1], [gardien2, defense2, demi2, attaque2]]

        Renvoie :
        	Booleen, tuple : renvoie vrai et le tuple correspondant a la position du joueur concerne par le rebond dans la liste joueurtot, ou faux et un tuple incoherent s'il n'y a pas d'impact.
        """

        xballe,yballe  = self.coord
        joueur_tot = joueurtot.joueurs_tot
        for i in range(2):
            for j in range(len(joueur_tot[0])):
                joueur = joueur_tot[i][j]
                abscisse = joueur.abs_joueurs
                for k in range(len(joueur.ordo_j)):
                    ordo = joueur.ordo_j[k]
                    d = np.sqrt((abscisse - xballe) ** 2 + (ordo - yballe) ** 2)
                    if d < joueur.rayon_pieds + self.rayon :
                        return True, (i,j,k)
        return False, (42, 42, 42)  #(42,42,42) ne sert a rien mais permet a la fonction de toujours etre BoolX(int tuple) en sortie

    # ==================================================================================================================
    def detectimpactmur(self,terrain):

        """
        Methode qui detecte un impact avec un mur selon la position de la balle.

        Parametre :
        	terrain (terrain) : terrain

        Renvoie :
        	Int : renvoie 1 s'il y a un impact avec un mur vertical, 0 s'il y a un impact avec un mur horizontal, et 2 s'il n'y a pas d'impact.
        """

        xballe, yballe = self.coord
        R = self.rayon
        longueur = terrain.longueur
        largeur = terrain.largeur

        if xballe < R:
            self.coord = (R, yballe)
            return 0
        if longueur - xballe < R:
            self.coord = (longueur-R, yballe)
            return 0
        if yballe < R :
            self.coord = (xballe, R)
            return 1
        if largeur - yballe < R:
            self.coord = (xballe, largeur - R)
            return 1
        return 2 #2 ne sert a rien pour le code mais permet a la fonction de toujours renvoyer un int


