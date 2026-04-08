# Prenotazione piazzole EV

Applicazione statica isolata per la prenotazione settimanale di 4 postazioni di ricarica: 3 T2 e 1 3A.

## Struttura

- `index.html`: interfaccia principale.
- `styles.css`: stile della pagina.
- `app.js`: logica di caricamento, validazione, rendering e sincronizzazione con Firebase.
- `firebase-config.js`: bootstrap Firebase Realtime Database per la cartella.
- `data/bookings.json`: file JSON locale usato come seed iniziale del database.

## Comportamento su GitHub Pages

GitHub Pages continua a servire solo file statici, ma le prenotazioni condivise vengono lette e scritte su Firebase Realtime Database.

Il file `data/bookings.json` viene usato solo per inizializzare il database la prima volta.

Dopo l'inizializzazione:

1. tutti i browser leggono le stesse prenotazioni da Firebase;
2. le modifiche vengono propagate in tempo reale senza login;
3. il JSON locale non viene piu usato come archivio operativo.

## Pubblicazione

Pubblica la cartella `ev-booking` nel repository dedicato oppure copiane il contenuto nella root del nuovo progetto GitHub Pages.

## Nota operativa

Per il funzionamento corretto servono anche regole Realtime Database che consentano lettura e scrittura pubblica per il nodo usato dall'app. Senza queste regole il frontend carichera Firebase ma non potra sincronizzare le prenotazioni.