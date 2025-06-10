enum mySerialPin {
    //% block="P0"
    P0 = SerialPin.P0,
    //% block="P1"
    P1 = SerialPin.P1,
    //% block="P2"
    P2 = SerialPin.P2
}
enum TrameType {
    //% block="GNGGA"
    GNGGA,
    //% block="GNGLL"
    GNGLL,
    //% block="GPGSA"
    GPGSA,
    //% block="BDGSA"
    BDGSA,
    //% block="GPGSV"
    GPGSV,
    //% block="BDGSV"
    BDGSV,
    //% block="GNRMC"
    GNRMC,
    //% block="GNVTG"
    GNVTG,
    //% block="GNZDA"
    GNZDA
}
enum ChampTypeGGA {
    /** Renvoie l'heure UTC au format HH:MM:SS.SSS*/
    //% block="HEURE UTC (HH:MM:SS.SSS)"
    HEURE,
    /** Renvoie la latitude en degré décimal*/
    //% block="LATITUDE (Degré Décimal)"
    LATITUDE,
    /** Renvoie la longitude en degré décimal*/
    //% block="LONGITUDE (Degré Décimal)"
    LONGITUDE,
    /** Renvoie la valeur du fix */
    //% block="FIX (0 : non valide, >=1 : valide)"
    FIX,
    /** Renvoie le nombre de satellites utilisés */
    //% block="NBSAT (nb de satelittes utilisés)"
    NBSAT,
    /** Renvoie l'altitude en mètre */
    //% block="ALTITUDE (en m)"
    ALTITUDE,
    /** Renvoie la précision horizontale en mètre */
    //% block="HDOP (précision lat/lon en m)"
    HDOP,
    /** Renvoie la précision verticale en mètre */
    //% block="VDOP (précision altitude en m)"
    VDOP
}
enum ChampTypeGLL {
    /** Renvoie l'heure UTC au format HH:MM:SS.SSS*/
    //% block="HEURE UTC (HH:MM:SS.SSS)"
    HEURE,
    /** Renvoie la latitude en degré décimal*/
    //% block="LATITUDE (Degré Décimal)"
    LATITUDE,
    /** Renvoie la longitude en degré décimal*/
    //% block="LONGITUDE (Degré Décimal)"
    LONGITUDE,
    /** Renvoie un indicateur de status ("A" : valide, "V": non valide) */
    //% block="ETAT ('A' : valide, 'V': non valide)"
    ETAT
}
enum ChampTypeGSA {
    /** Renvoie le type de mode "A" : Automatic ou "M" : Manuel*/
    //% block="MODE ('A' : Automatic, 'M' : Manuel)"
    MODE,
    /** Renvoie le type de fix (1 : Not available, 2: 2D, 3 : 3D) */
    //% block="TYPE FIX (1 : Not available, 2: 2D, 3 : 3D)"
    FONCTIONNEMENT,
    /** Renvoie les valeurs des identifiants PRN des satellites utilisés */
    //% block="Identifiants PRN des Satellites utilisés"
    IDSAT,
    /** Renvoie la précision globale en mètre */
    //% block="PDOP (précision globale en m)"
    PDOP,
    /** Renvoie la précision horizontale en mètre */
    //% block="HDOP (précision lat/lon en m)"
    HDOP,
    /** Renvoie la précision verticale en mètre */
    //% block="VDOP (précision altitude en m)"
    VDOP
}
enum ChampTypeGSV {
    /** Renvoie le nombre de satellites utilisés */
    //% block="NBSAT (nb de satelittes utilisés)"
    NBSAT,
    /** Renvoie la valeur de l'identifiant PRN du satellite choisi */
    //% block="Identifiants PRN du Satellite choisi"
    ID,
    /** Renvoie la valeur de l'élévation du satellite choisi (en Degré) */
    //% block="ELEVATION du Satellite choisi (en Degré)"
    ELEVATION,
    /** Renvoie la valeur de l'azimut du satellite choisi (en Degré) */
    //% block="AZIMUT du Satellite choisi (en Degré)"
    AZIMUT,
    /** Renvoie la valeur de la force de signal du satellite choisi (0 à 90 dB) */
    //% block="FORCE du signal du Satellite choisi (0 à 90dB)"
    FORCE,
}
enum ChampTypeRMC {
    /** Renvoie l'heure UTC au format HH:MM:SS.SSS*/
    //% block="HEURE UTC (HH:MM:SS.SSS)"
    HEURE,
    /** Renvoie la latitude en degré décimal*/
    //% block="LATITUDE (Degré Décimal)"
    LATITUDE,
    /** Renvoie la longitude en degré décimal*/
    //% block="LONGITUDE (Degré Décimal)"
    LONGITUDE,
    /** Renvoie un indicateur de status ("A" : valide, "V": non valide) */
    //% block="ETAT ('A' : valide, 'V': non valide)"
    ETAT,
    /** Renvoie la vitesse au sol en noeuds*/
    //% block="VITESSE (en Noeuds)"
    VITESSE,
    /** Renvoie la Route (Direction au sol parcouru en degré)*/
    //% block="ROUTE (Direction au sol parcouru en degré)"
    ROUTE,
    /** Renvoie la date UTC (JJ/MM/AA)*/
    //% block="DATE UTC (JJ/MM/AA)"
    DATE
}
enum ChampTypeVTG {
    /** Renvoie le CAP (Direction au sol actuel en degré)*/
    //% block="CAP (Direction au sol actuel en degré)"
    CAP,
    /** Renvoie la vitesse au sol en noeuds*/
    //% block="VITESSE (en Noeuds)"
    VITESSENOEUD = 2,
    /** Renvoie la vitesse au sol en km/h*/
    //% block="VITESSE (en km/h)"
    VITESSEKMH
}
enum ChampTypeZDA {
    /** Renvoie l'heure UTC au format HH:MM:SS.SSS*/
    //% block="HEURE UTC (HH:MM:SS.SSS)"
    HEURE,
    /** Renvoie la valeur du jour UTC entre 1 et 31*/
    //% block="JOUR UTC (entre 1 et 31)"
    JOUR,
    /** Renvoie la valeur du mois UTC entre 1 et 12*/
    //% block="MOIS UTC (entre 1 et 12)"
    MOIS,
    /** Renvoie la valeur de l'année UTC entre 1 et 12*/
    //% block="ANNEE (AAAA)"
    ANNEE
}
enum ChampTypeSTR {
    /** Renvoie l'heure UTC au format HH:MM:SS.SSS*/
    //% block="HEURE UTC (HH:MM:SS.SSS)"
    HEURE,
    /** Renvoie un indicateur de status ("A" : valide, "V": non valide) */
    //% block="ETAT ('A' : valide, 'V': non valide)"
    ETAT,
    /** Renvoie le type de mode "A" : Automatic ou "M" : Manuel*/
    //% block="MODE ('A' : Automatic, 'M' : Manuel)"
    MODE,
    /** Renvoie la date UTC (JJ/MM/AA)*/
    //% block="DATE UTC (JJ/MM/AA)"
    DATE
}
enum ChampTypeNUM {
    /** Renvoie la latitude en degré décimal*/
    //% block="LATITUDE (Degré Décimal)"
    LATITUDE,
    /** Renvoie la longitude en degré décimal*/
    //% block="LONGITUDE (Degré Décimal)"
    LONGITUDE,
    /** Renvoie la valeur du fix */
    //% block="FIX (0 : non valide, >=1 : valide)"
    FIX,
    /** Renvoie le type de fix (1 : Not available, 2: 2D, 3 : 3D) */
    //% block="TYPE FIX (1 : Not available, 2: 2D, 3 : 3D)"
    FONCTIONNEMENT,
    /** Renvoie le nombre de satellites utilisés */
    //% block="NBSAT (nb de satelittes utilisés)"
    NBSAT,
    /** Renvoie l'altitude en mètre */
    //% block="ALTITUDE (en m)"
    ALTITUDE,
    /** Renvoie la précision horizontale en mètre */
    //% block="HDOP (précision lat/lon en m)"
    HDOP,
    /** Renvoie la précision verticale en mètre */
    //% block="VDOP (précision altitude en m)"
    VDOP,
    /** Renvoie la précision globale en mètre */
    //% block="PDOP (précision globale en m)"
    PDOP,
    /** Renvoie la vitesse au sol en noeuds*/
    //% block="VITESSE (en Noeuds)"
    VITESSENOEUD,
    /** Renvoie la vitesse au sol en km/h*/
    //% block="VITESSE (en km/h)"
    VITESSEKMH,
    /** Renvoie la Route (Direction au sol parcouru en degré)*/
    //% block="ROUTE (Direction au sol parcouru en degré)"
    ROUTE,
    /** Renvoie le CAP (Direction au sol actuel en degré)*/
    //% block="CAP (Direction au sol actuel en degré)"
    CAP,
    /** Renvoie la valeur du jour UTC entre 1 et 31*/
    //% block="JOUR UTC (entre 1 et 31)"
    JOUR,
    /** Renvoie la valeur du mois UTC entre 1 et 12*/
    //% block="MOIS UTC (entre 1 et 12)"
    MOIS,
    /** Renvoie la valeur de l'année UTC entre 1 et 12*/
    //% block="ANNEE (AAAA)"
    ANNEE
}

enum ChampTypeList {
    /** Renvoie la valeur de l'identifiant PRN du satellite choisi */
    //% block="Identifiants PRN du Satellite choisi"
    ID,
    /** Renvoie la valeur de l'élévation du satellite choisi (en Degré) */
    //% block="ELEVATION du Satellite choisi (en Degré)"
    ELEVATION,
    /** Renvoie la valeur de l'azimut du satellite choisi (en Degré) */
    //% block="AZIMUT du Satellite choisi (en Degré)"
    AZIMUT,
    /** Renvoie la valeur de la force de signal du satellite choisi (0 à 90 dB) */
    //% block="FORCE du signal du Satellite choisi (0 à 90dB)"
    FORCE,
}


namespace GPS_AT6558 {

    function toSerialPin(pin: mySerialPin): SerialPin {
        return pin as any as SerialPin
    }

    function nomTrame(t: TrameType): string {
        switch (t) {
            case TrameType.GNGGA: return "GNGGA"
            case TrameType.GNGLL: return "GNGLL"
            case TrameType.GPGSA: return "GPGSA"
            case TrameType.BDGSA: return "BDGSA"
            case TrameType.GPGSV: return "GPGSV"
            case TrameType.BDGSV: return "BDGSV"
            case TrameType.GNRMC: return "GNRMC"
            case TrameType.GNVTG: return "GNVTG"
            case TrameType.GNZDA: return "GNZDA"
            default: return ""
        }
    }

    /**
        *Initialise le GPS sur la broche choisie                  
    */
    //% block="initialiser GPS sur broche %rx (sans maj auto)"
    //% rx.fieldEditor="gridpicker"
    //% rx.fieldOptions.columns=3
    //% rx.defl=mySerialPin.P0
    //% weight=20
    //% subcategory="Avancé : Trames NMEA"
    export function initGps(rx: mySerialPin): void {
        serial.redirect(SerialPin.USB_TX, toSerialPin(rx), 9600)
        serial.setTxBufferSize(128)
        serial.setRxBufferSize(128)
        serial.readUntil("\n") // nettoyer une trame éventuelle en attente
    }

    /**
        Retourne une chaîne de caractère correspondante au type de trame NMEA choisie.
    **/
    //% block="Récupérer une trame $identifiant avec un timeout de $timeout"
    //% identifiant.defl=TrameType.GNGGA
    //% timeout.defl=5000 
    //% weight=19
    //% subcategory="Avancé : Trames NMEA"
    //% inlineInputMode=external
    export function getTrameNMEA(identifiant: TrameType, timeout: number): string {
        let idTrame = nomTrame(identifiant)
        let start = control.millis()

        // Si ce sont des trames GSV, on prépare à les collecter toutes
        if (idTrame == "GPGSV" || idTrame == "BDGSV") {
            let trames: string[] = []
            let compteur = 0
            while (control.millis() - start < 3 * timeout) {
                const trame = serial.readUntil("\n").trim()
                if (trame && trame.includes(",")) {
                    let champs = trame.split(",")
                    let id = champs[0]
                    if (id == "$" + idTrame) {
                        let numTrame = parseInt(champs[2])      // numéro de cette trame (1, 2, ...)
                        let totalTrames = parseInt(champs[1])   // nombre total de trames

                        if (!trames[numTrame - 1]) {
                            trames[numTrame - 1] = trame         // on place la trame à la bonne position
                            compteur += 1
                        }

                        // Si on a reçu toutes les trames, on les renvoie dans l'ordre
                        if (compteur == totalTrames) {
                            return trames.join("@")
                        }
                    }
                }
                basic.pause(10)
            }
            return trames.join("@") // Même si incomplètes
        }
        while (control.millis() - start < timeout) {
            const trame2 = serial.readUntil("\n").trim()
            if (trame2 && trame2.includes(",")) {
                let id2 = trame2.split(",")[0]
                if (id2 == "$" + idTrame) {
                    return trame2
                }
            }
            basic.pause(10)
        }
        return ""
    }

    /** 
        Retourne une valeur numérique du checksum (0–255) d'une trame
    **/
    //% block="Calcul Checksum d'une trame $trame"
    //% weight=3
    //% subcategory="Checksum"
    export function calculateChecksum(trame: string): number {
        // Vérifie que la trame commence bien par '$' et contient '*'
        if (trame.charAt(0) != "$" || trame.indexOf("*") == -1) {
            return -1
        }
        // Extrait la portion entre $ et *
        let start2 = 1
        let end = trame.indexOf("*")
        let data = trame.slice(start2, end)

        // Calcule le XOR de tous les caractères
        let checksum = 0
        for (let i = 0; i < data.length; i++) {
            checksum ^= data.charCodeAt(i)
        }
        return checksum
    }

    /** 
        Retourne un booléen indiquant si le checksum de la trame est bon
    **/
    //% block="Validation d'une trame $trame"
    //% weight=2
    //% subcategory="Checksum"
    export function checkTrame(trame: string): boolean {
        let sepIndex = trame.indexOf("*")
        if (trame.charAt(0) != "$" || sepIndex == -1) {
            return false
        }
        let expected = parseInt(trame.slice(sepIndex + 1, sepIndex + 3), 16)
        let actual = calculateChecksum(trame)
        return expected == actual
    }

    /** Retourne une chaîne de caractère du champ chosie de la trame GNGGA.*/
    //% block="Lire $champ dans la trame GNGGA $trame"
    //% champ.defl=ChampTypeGGA.HEURE
    //% trame.defl=""
    //% weight=18
    //% subcategory="Avancé : Trames NMEA"
    //% inlineInputMode=external
    export function getValueGNGGA(champ: ChampTypeGGA, trame: string): string {
        if (trame == "" || !checkTrame(trame)) {
            return ""
        }
        let champs2 = trame.split(",")
        switch (champ) {
            case ChampTypeGGA.HEURE:
                return (champs2[1].slice(0, 2) + ":" + champs2[1].slice(2, 4) + ":" + champs2[1].slice(4))
            case ChampTypeGGA.LATITUDE:
                let degLat = parseFloat(champs2[2].slice(0, 2))
                let minLat = parseFloat(champs2[2].slice(2))
                let latitude = degLat + minLat / 60
                if (champs2[3] == "S") {
                    latitude = -latitude
                }
                return latitude.toString()
            case ChampTypeGGA.LONGITUDE:
                let degLon = parseFloat(champs2[4].slice(0, 3))
                let minLon = parseFloat(champs2[4].slice(3))
                let longitude = degLon + minLon / 60
                if (champs2[5] == "W") {
                    longitude = -longitude
                }
                return longitude.toString()
            case ChampTypeGGA.FIX:
                return champs2[6]
            case ChampTypeGGA.NBSAT:
                return champs2[7]
            case ChampTypeGGA.ALTITUDE:
                return champs2[9]
            case ChampTypeGGA.HDOP:
                return champs2[8]
            case ChampTypeGGA.VDOP:
                return champs2[11]
            default:
                return ""
        }
    }

    /** Retourne une chaîne de caractère du champ chosie de la trame GNGLL.*/
    //% block="Lire $champ dans la trame GNGLL $trame"
    //% champ.defl=ChampTypeGLL.ETAT
    //% weight=17
    //% subcategory="Avancé : Trames NMEA"
    //% inlineInputMode=external
    export function getValueGNGLL(champ: ChampTypeGLL, trame: string): string {
        if (trame == "" || !checkTrame(trame)) {
            return ""
        }
        let champs3 = trame.split(",")
        switch (champ) {
            case ChampTypeGLL.HEURE:
                return (champs3[5].slice(0, 2) + ":" + champs3[5].slice(2, 4) + ":" + champs3[5].slice(4))
            case ChampTypeGLL.LATITUDE:
                let degLat2 = parseFloat(champs3[1].slice(0, 2))
                let minLat2 = parseFloat(champs3[1].slice(2))
                let latitude2 = degLat2 + minLat2 / 60
                if (champs3[2] == "S") {
                    latitude2 = -latitude2
                }
                return latitude2.toString()
            case ChampTypeGLL.LONGITUDE:
                let degLon2 = parseFloat(champs3[3].slice(0, 3))
                let minLon2 = parseFloat(champs3[3].slice(3))
                let longitude2 = degLon2 + minLon2 / 60
                if (champs3[4] == "W") {
                    longitude2 = -longitude2
                }
                return longitude2.toString()
            case ChampTypeGLL.ETAT:
                return champs3[6]
            default:
                return ""
        }
    }

    /** Retourne une chaîne de caractère du champ chosie de la trame GPGSA ou BDGSA.*/
    //% block="Lire $champ dans la trame GPGSA ou BDGSA $trame"
    //% champ.defl=ChampTypeGSA.SELECTION
    //% weight=16
    //% subcategory="Avancé : Trames NMEA"
    //% inlineInputMode=external
    export function getValueGPGSABDGSA(champ: ChampTypeGSA, trame: string): string {
        if (trame == "" || !checkTrame(trame)) {
            return ""
        }
        let champs4 = trame.split(",")
        switch (champ) {
            case ChampTypeGSA.MODE:
                return champs4[1]
            case ChampTypeGSA.FONCTIONNEMENT:
                return champs4[2]
            case ChampTypeGSA.IDSAT:
                let id3 = ""
                for (let j = 3; j < 14; j++) {
                    if (champs4[j] != "") {
                        id3 = id3 + champs4[j] + " "
                    }
                }
                id3.trim()
                return id3
            case ChampTypeGSA.PDOP:
                return champs4[15]
            case ChampTypeGSA.HDOP:
                return champs4[16]
            case ChampTypeGSA.VDOP:
                let end2 = champs4[17].indexOf("*")
                return champs4[17].slice(0, end2)
            default:
                return ""
        }
    }

    /** Retourne une chaîne de caractère du champ chosie de la trame GPGS ou BDGSV.*/
    //% block="Lire $champ dans la trame GPGSV ou BDGSV $trame du satellite $numero"
    //% champ.defl=ChampTypeGSV.ID
    //% numero.defl=1
    //% weight=15
    //% subcategory="Avancé : Trames NMEA"
    //% inlineInputMode=external
    export function getValueGPGSVBDGSV(champ: ChampTypeGSV, trame: string, numero: number): string {
        let trames2 = trame.split("@")
        for (let k = 0; k < trames2.length; k++) {
            if (trames2[k] == "" || !checkTrame(trames2[k])) {
                return ""
            }
        }
        switch (champ) {
            case ChampTypeGSV.NBSAT:
                return trames2[0].split(",")[3]
            case ChampTypeGSV.ID:
                if (numero < 1 || numero > parseInt(trames2[0].split(",")[3])) {
                    return ""
                }
                else {
                    return trames2[Math.floor((numero - 1) / 4)].split(",")[4 + (numero - 1) % 4 * 4]
                }
            case ChampTypeGSV.ELEVATION:
                if (numero < 1 || numero > parseInt(trames2[0].split(",")[3])) {
                    return ""
                }
                else {
                    return trames2[Math.floor((numero - 1) / 4)].split(",")[5 + (numero - 1) % 4 * 4]
                }
            case ChampTypeGSV.AZIMUT:
                if (numero < 1 || numero > parseInt(trames2[0].split(",")[3])) {
                    return ""
                }
                else {
                    return trames2[Math.floor((numero - 1) / 4)].split(",")[6 + (numero - 1) % 4 * 4]
                }
            case ChampTypeGSV.FORCE:
                if (numero < 1 || numero > parseInt(trames2[0].split(",")[3])) {
                    return ""
                }
                else {
                    let force = trames2[Math.floor((numero - 1) / 4)].split(",")[7 + (numero - 1) % 4 * 4]
                    let pos = force.indexOf("*")
                    if (pos == -1) {
                        return force
                    }
                    else {
                        return force.slice(0, pos)
                    }

                }
            default:
                return ""
        }
    }

    /** Retourne une chaîne de caractère du champ chosie de la trame GNRMC.*/
    //% block="Lire $champ dans la trame GNRMC $trame"
    //% champ.defl=ChampTypeRMC.ROUTE
    //% weight=14
    //% subcategory="Avancé : Trames NMEA"
    //% inlineInputMode=external
    export function getValueGNRMC(champ: ChampTypeRMC, trame: string): string {
        if (trame == "" || !checkTrame(trame)) {
            return ""
        }
        let champs5 = trame.split(",")
        switch (champ) {
            case ChampTypeRMC.HEURE:
                return (champs5[1].slice(0, 2) + ":" + champs5[1].slice(2, 4) + ":" + champs5[1].slice(4))
            case ChampTypeRMC.LATITUDE:
                let degLat3 = parseFloat(champs5[3].slice(0, 2))
                let minLat3 = parseFloat(champs5[3].slice(2))
                let latitude3 = degLat3 + minLat3 / 60
                if (champs5[4] == "S") {
                    latitude3 = -latitude3
                }
                return latitude3.toString()
            case ChampTypeRMC.LONGITUDE:
                let degLon3 = parseFloat(champs5[5].slice(0, 3))
                let minLon3 = parseFloat(champs5[5].slice(3))
                let longitude3 = degLon3 + minLon3 / 60
                if (champs5[6] == "W") {
                    longitude3 = -longitude3
                }
                return longitude3.toString()
            case ChampTypeRMC.ETAT:
                return champs5[2]
            case ChampTypeRMC.VITESSE:
                return champs5[7]
            case ChampTypeRMC.ROUTE:
                return champs5[8]
            case ChampTypeRMC.DATE:
                return (champs5[9].slice(0, 2) + "/" + champs5[9].slice(2, 4) + "/" + champs5[9].slice(4))
            default:
                return ""
        }

    }

    /** Retourne une chaîne de caractère du champ chosie de la trame GNVTG.*/
    //% block="Lire $champ dans la trame GNVTG $trame"
    //% champ.defl=ChampTypeVTG.CAP
    //% weight=13
    //% subcategory="Avancé : Trames NMEA"
    //% inlineInputMode=external
    export function getValueGNVTG(champ: ChampTypeVTG, trame: string): string {
        if (trame == "" || !checkTrame(trame)) {
            return ""
        }
        let champs6 = trame.split(",")
        switch (champ) {
            case ChampTypeVTG.CAP:
                return champs6[1]
            case ChampTypeVTG.VITESSENOEUD:
                return champs6[5]
            case ChampTypeVTG.VITESSEKMH:
                return champs6[7]
            default:
                return ""
        }
    }

    /** Retourne une chaîne de caractère du champ chosie de la trame GNZDA.*/
    //% block="Lire $champ dans la trame GNZDA $trame"
    //% champ.defl=ChampTypeZDA.JOUR
    //% weight=12
    //% subcategory="Avancé : Trames NMEA"
    //% inlineInputMode=external
    export function getValueGNZDA(champ: ChampTypeZDA, trame: string): string {
        if (trame == "" || !checkTrame(trame)) {
            return ""
        }
        let champs7 = trame.split(",")
        switch (champ) {
            case ChampTypeZDA.HEURE:
                return (champs7[1].slice(0, 2) + ":" + champs7[1].slice(2, 4) + ":" + champs7[1].slice(4))
            case ChampTypeZDA.JOUR:
                return champs7[2]
            case ChampTypeZDA.MOIS:
                return champs7[3]
            case ChampTypeZDA.ANNEE:
                return champs7[4]
            default:
                return ""
        }
    }

    /** Retourne un booléen indiquant si le module récupère des coordonnées.*/
    //% block="Module prêt ? "
    //% weight=99
    //% group="Initialisation du module"
    export function hasFix(): boolean {
        let trame3 = getTrameNMEA(TrameType.GNGGA, 5000)
        let fixStr = getValueGNGGA(ChampTypeGGA.FIX, trame3)
        let fix = parseInt(fixStr)
        let trame22 = getTrameNMEA(TrameType.GNRMC, 5000)
        let etat = getValueGNRMC(ChampTypeRMC.ETAT, trame22)
        let trame32 = getTrameNMEA(TrameType.GNGLL, 5000)
        let etat2 = getValueGNGLL(ChampTypeGLL.ETAT, trame32)
        return !isNaN(fix) && fix >= 1 && etat == "A" && etat2 == "A"
    }



    class GPS {
        private latitude: number
        private longitude: number
        private fix: number
        private nbsat: number
        private altitude: number
        private hdop: number
        private vdop: number
        private pdop: number
        private vitesseNoeuds: number
        private vitesseKmH: number
        private route: number
        private cap: number
        private jour: number
        private mois: number
        private annee: number
        private fonctionnement: number

        private heureUTC: string
        private date: string
        private etat: string
        private mode: string

        private taille: number
        private ids: number[]
        private elevations: number[]
        private azimuts: number[]
        private forces: number[]

        private enMarche: boolean

        constructor() {
            this.enMarche = false
            this.latitude = NaN
            this.longitude = NaN
            this.fix = NaN
            this.nbsat = NaN
            this.altitude = NaN
            this.hdop = NaN
            this.vdop = NaN
            this.pdop = NaN
            this.vitesseNoeuds = NaN
            this.vitesseKmH = NaN
            this.route = NaN
            this.cap = NaN
            this.jour = NaN
            this.mois = NaN
            this.annee = NaN
            this.fonctionnement = NaN

            this.heureUTC = ""
            this.date = ""
            this.etat = ""
            this.mode = ""
            this.taille = 24
            this.ids = []
            this.elevations = []
            this.azimuts = []
            this.forces = []
            for (let i = 0; i < this.taille; i++) {
                this.ids.push(NaN)
                this.elevations.push(NaN)
                this.azimuts.push(NaN)
                this.forces.push(NaN)
            }



        }
        public demarrer(): void {
            this.enMarche = true
            control.inBackground(() => {
                while (this.enMarche) {
                    let trame4 = getTrameNMEA(TrameType.GNGGA, 2000)
                    if (trame4 != "") {
                        this.heureUTC = getValueGNGGA(ChampTypeGGA.HEURE, trame4)
                        this.latitude = parseFloat(getValueGNGGA(ChampTypeGGA.LATITUDE, trame4))
                        this.longitude = parseFloat(getValueGNGGA(ChampTypeGGA.LONGITUDE, trame4))
                        this.fix = parseInt(getValueGNGGA(ChampTypeGGA.FIX, trame4))
                        this.nbsat = parseInt(getValueGNGGA(ChampTypeGGA.NBSAT, trame4))
                        this.altitude = parseFloat(getValueGNGGA(ChampTypeGGA.ALTITUDE, trame4))
                        this.hdop = parseFloat(getValueGNGGA(ChampTypeGGA.HDOP, trame4))
                        this.vdop = parseFloat(getValueGNGGA(ChampTypeGGA.VDOP, trame4))
                    }
                    trame4 = getTrameNMEA(TrameType.GPGSA, 2000)
                    if (trame4 != "") {
                        this.mode = getValueGPGSABDGSA(ChampTypeGSA.MODE, trame4)
                        this.fonctionnement = parseInt(getValueGPGSABDGSA(ChampTypeGSA.FONCTIONNEMENT, trame4))
                        this.pdop = parseFloat(getValueGPGSABDGSA(ChampTypeGSA.PDOP, trame4))
                    }
                    trame4 = getTrameNMEA(TrameType.GNRMC, 2000)
                    if (trame4 != "") {
                        this.date = getValueGNRMC(ChampTypeRMC.DATE, trame4)
                        this.route = parseFloat(getValueGNRMC(ChampTypeRMC.ROUTE, trame4))
                        this.etat = getValueGNRMC(ChampTypeRMC.ETAT, trame4)
                        this.vitesseNoeuds = parseFloat(getValueGNRMC(ChampTypeRMC.VITESSE, trame4))
                    }
                    trame4 = getTrameNMEA(TrameType.GNVTG, 2000)
                    if (trame4 != "") {
                        this.cap = parseFloat(getValueGNVTG(ChampTypeVTG.CAP, trame4))
                        this.vitesseKmH = parseFloat(getValueGNVTG(ChampTypeVTG.VITESSEKMH, trame4))

                    }
                    trame4 = getTrameNMEA(TrameType.GNZDA, 2000)
                    if (trame4 != "") {
                        this.jour = parseInt(getValueGNZDA(ChampTypeZDA.JOUR, trame4))
                        this.mois = parseInt(getValueGNZDA(ChampTypeZDA.MOIS, trame4))
                        this.annee = parseInt(getValueGNZDA(ChampTypeZDA.ANNEE, trame4))
                    }
                    trame4 = getTrameNMEA(TrameType.BDGSV, 2000)
                    let nb = 0
                    if (trame4 != "") {
                        nb = parseInt(getValueGPGSVBDGSV(ChampTypeGSV.NBSAT, trame4, 1))
                        for (let i = 0; i < nb; i++) {
                            this.ids[i] = parseInt(getValueGPGSVBDGSV(ChampTypeGSV.ID, trame4, i + 1))
                            this.elevations[i] = parseFloat(getValueGPGSVBDGSV(ChampTypeGSV.ELEVATION, trame4, i + 1))
                            this.forces[i] = parseFloat(getValueGPGSVBDGSV(ChampTypeGSV.FORCE, trame4, i + 1))
                            this.azimuts[i] = parseFloat(getValueGPGSVBDGSV(ChampTypeGSV.AZIMUT, trame4, i + 1))
                        }
                    }
                    trame4 = getTrameNMEA(TrameType.GPGSV, 2000)
                    if (trame4 != "") {
                        let nb2 = parseInt(getValueGPGSVBDGSV(ChampTypeGSV.NBSAT, trame4, 1))
                        for (let i = 0; i < nb2; i++) {
                            this.ids[nb + i] = parseInt(getValueGPGSVBDGSV(ChampTypeGSV.ID, trame4, i + 1))
                            this.elevations[nb + i] = parseFloat(getValueGPGSVBDGSV(ChampTypeGSV.ELEVATION, trame4, i + 1))
                            this.forces[nb + i] = parseFloat(getValueGPGSVBDGSV(ChampTypeGSV.FORCE, trame4, i + 1))
                            this.azimuts[nb + i] = parseFloat(getValueGPGSVBDGSV(ChampTypeGSV.AZIMUT, trame4, i + 1))
                        }
                    }
                }
            })
        }
        public getLatitude(): number {
            return this.latitude
        }
        public getLongitude(): number {
            return this.longitude
        }
        public getFix(): number {
            return this.fix
        }
        public getNbSat(): number {
            return this.nbsat
        }
        public getAltitude(): number {
            return this.altitude
        }
        public getHDOP(): number {
            return this.hdop
        }
        public getVDOP(): number {
            return this.vdop
        }
        public getPDOP(): number {
            return this.pdop
        }
        public getVitesseN(): number {
            return this.vitesseNoeuds
        }
        public getVitesseK(): number {
            return this.vitesseKmH
        }
        public getRoute(): number {
            return this.route
        }
        public getCap(): number {
            return this.cap
        }
        public getJour(): number {
            return this.jour
        }
        public getMois(): number {
            return this.mois
        }
        public getAnnee(): number {
            return this.annee
        }
        public getFonctionnement(): number {
            return this.fonctionnement
        }
        public getHeure(): string {
            return this.heureUTC
        }
        public getDate(): string {
            return this.date
        }
        public getEtat(): string {
            return this.etat
        }
        public getMode(): string {
            return this.mode
        }
        public getId(nombre: number): number {
            return this.ids[nombre]
        }
        public getElevation(nombre: number): number {
            return this.elevations[nombre]
        }
        public getAzimut(nombre: number): number {
            return this.azimuts[nombre]
        }
        public getForces(nombre: number): number {
            return this.forces[nombre]
        }
        public arreter() {
            this.enMarche = false
        }
    }

    let gps = new GPS()
    // Fonctions exportées vers MakeCode (sous forme de blocs)

    /**Initialise le GPS sur la broche choisie*/
    //% block="initialiser GPS sur broche %rx"
    //% rx.fieldEditor="gridpicker"
    //% rx.fieldOptions.columns=3
    //% rx.defl=mySerialPin.P0
    //% weight=100
    //% group="Initialisation du module"
    export function initGpsv2(rx: mySerialPin): void {
        serial.redirect(SerialPin.USB_TX, toSerialPin(rx), 9600)
        serial.setTxBufferSize(128)
        serial.setRxBufferSize(128)
        serial.readUntil("\n") // nettoyer une trame éventuelle
        gps.demarrer()
    }

    /** Retourne une chaîne de caractère du champ chosie*/
    //% block="Lire chaîne $champ"
    //% champ.defl=ChampTypeSTR.HEURE
    //% weight=99
    //% group="GPS"
    export function getValueStr(champ: ChampTypeSTR): string {
        switch (champ) {
            case ChampTypeSTR.HEURE:
                return gps.getHeure()
            case ChampTypeSTR.ETAT:
                return gps.getEtat()
            case ChampTypeSTR.MODE:
                return gps.getMode()
            case ChampTypeSTR.DATE:
                return gps.getDate()
            default:
                return ""
        }
    }

    /** Retourne la valeur du nombre du champ chosie*/
    //% block="Lire nombre $champ"
    //% champ.defl=ChampTypeNUM.LATITUDE
    //% weight=98
    //% group="GPS"
    export function getValueNbr(champ: ChampTypeNUM): number {
        switch (champ) {
            case ChampTypeNUM.LATITUDE:
                return gps.getLatitude()
            case ChampTypeNUM.LONGITUDE:
                return gps.getLongitude()
            case ChampTypeNUM.FIX:
                return gps.getFix()
            case ChampTypeNUM.NBSAT:
                return gps.getNbSat()
            case ChampTypeNUM.ALTITUDE:
                return gps.getAltitude()
            case ChampTypeNUM.HDOP:
                return gps.getHDOP()
            case ChampTypeNUM.VDOP:
                return gps.getVDOP()
            case ChampTypeNUM.PDOP:
                return gps.getPDOP()
            case ChampTypeNUM.VITESSENOEUD:
                return gps.getVitesseN()
            case ChampTypeNUM.VITESSEKMH:
                return gps.getVitesseK()
            case ChampTypeNUM.ROUTE:
                return gps.getRoute()
            case ChampTypeNUM.CAP:
                return gps.getCap()
            case ChampTypeNUM.JOUR:
                return gps.getJour()
            case ChampTypeNUM.MOIS:
                return gps.getMois()
            case ChampTypeNUM.ANNEE:
                return gps.getAnnee()
            case ChampTypeNUM.FONCTIONNEMENT:
                return gps.getFonctionnement()
            default:
                return NaN
        }
    }

    /** Retourne la valeur du nombre du champ chosie*/
    //% block="Lire nombre $champ du satellite $numero"
    //% champ.defl=ChampTypeList.ID
    //% numero.defl=1
    //% weight=97
    //% group="GPS"
    export function getValueList(champ: ChampTypeList, numero: number): number {
        switch (champ) {
            case ChampTypeList.ID:
                return gps.getId(numero - 1)
            case ChampTypeList.ELEVATION:
                return gps.getElevation(numero - 1)
            case ChampTypeList.AZIMUT:
                return gps.getAzimut(numero - 1)
            case ChampTypeList.FORCE:
                return gps.getForces(numero - 1)
            default:
                return NaN
        }
    }

    //% block="Arrêter GPS"
    //% group="Initialisation du module"
    //% weight=96
    export function arreterGPS() {
        gps.arreter()
    }



    /** Retourne la valeur numérique de la distance entre le point A et le point B*/
    //% block="Distance entre |A $latA $longA et |B $latB $longB (en m)"
    //% latA.defl=49.2147928
    //% longA.defl=-0.3675902
    //% latB.defl=46.6703886
    //% longB.defl=0.3701164
    //% weight=99
    //% group="Cap et distance"
    //% inlineInputMode=external
    export function calcul_distance(latA: number, longA: number, latB: number, longB: number): number {
        latA = latA * Math.PI / 180
        longA = longA * Math.PI / 180
        latB = latB * Math.PI / 180
        longB = longB * Math.PI / 180
        let R = 6372795.477598
        let distance = R * Math.acos(Math.sin(latA) * Math.sin(latB) + Math.cos(latA) * Math.cos(latB) * Math.cos(longA - longB))
        return Math.trunc(distance)
    }

    /** Affiche une flèche en fonction de l'angle passé en argument*/
    //% block="Affiche une flèche en fonction de l'angle $angle"
    //% weight=98
    //% group="Cap et distance"
    //% inlineInputMode=external
    export function affichage(angle: number): void {
        if (angle <= 22.5 || angle >= 337.5) {
            basic.showArrow(ArrowNames.North)
        } else if (angle <= 67.5) {
            basic.showArrow(ArrowNames.NorthEast)
        } else if (angle <= 112.5) {
            basic.showArrow(ArrowNames.East)
        } else if (angle <= 157.5) {
            basic.showArrow(ArrowNames.SouthEast)
        } else if (angle <= 202.5) {
            basic.showArrow(ArrowNames.South)
        } else if (angle <= 247.5) {
            basic.showArrow(ArrowNames.SouthWest)
        } else if (angle <= 292.5) {
            basic.showArrow(ArrowNames.West)
        } else if (angle < 337.5) {
            basic.showArrow(ArrowNames.NorthWest)
        }
    }

    /** Retourne la valeur numérique du cap en degré entre la coordonnée A et B*/
    //% block="Cap entre |A $latA $longA et |B $latB $longB (em degré)"
    //% latA.defl=49.2147928
    //% longA.defl=-0.3675902
    //% latB.defl=46.6703886
    //% longB.defl=0.3701164
    //% weight=100
    //% group="Cap et distance"
    //% inlineInputMode=external
    export function calcul_cap(latA: number, longA: number, latB: number, longB: number): number {
        latA = latA * Math.PI / 180
        longA = longA * Math.PI / 180
        latB = latB * Math.PI / 180
        longB = longB * Math.PI / 180
        let phi = Math.log(Math.tan(latB / 2 + Math.PI / 4) / Math.tan(latA / 2 + Math.PI / 4))
        let lo = longB - longA
        let roulement = (Math.atan2(lo, phi) * 360 / (2 * Math.PI) + 360) % 360
        return roulement
    }

}