# Prenotazione piazzole EV

Applicazione statica isolata per la prenotazione settimanale di 4 postazioni di ricarica: 3 T2 e 1 3A.

## Struttura

- `index.html`: interfaccia principale.
- `styles.css`: stile della pagina.
- `app.js`: logica di caricamento, validazione, rendering e import/export JSON.
- `data/bookings.json`: file JSON locale usato come base dati iniziale.

## Comportamento su GitHub Pages

GitHub Pages serve file statici: il browser puo leggere `data/bookings.json`, ma non puo riscriverlo sul server.

Per questo l'app fa tre cose:

1. carica `data/bookings.json` al primo avvio;
2. salva le modifiche nel `localStorage` del browser;
3. consente di scaricare un nuovo `bookings.json` da pubblicare nel repository.

## Pubblicazione

Pubblica la cartella `ev-booking` nel repository dedicato oppure copiane il contenuto nella root del nuovo progetto GitHub Pages.