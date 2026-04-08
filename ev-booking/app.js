const STORAGE_KEY = "ev-booking-state-v5";
const DATA_URL = "./data/bookings.json";
const DAYS = ["Lunedi", "Martedi", "Mercoledi", "Giovedi", "Venerdi", "Sabato"];

const state = {
  data: null,
  activeWeekStart: null,
  selectedStationId: null,
  pendingBooking: null,
};

const elements = {
  weekPicker: document.querySelector("#week-picker"),
  weekRange: document.querySelector("#week-range"),
  feedback: document.querySelector("#feedback"),
  bookingCount: document.querySelector("#booking-count"),
  overviewGrid: document.querySelector("#overview-grid"),
  selectedStationName: document.querySelector("#selected-station-name"),
  selectedStationMeta: document.querySelector("#selected-station-meta"),
  calendarGrid: document.querySelector("#calendar-grid"),
  selectedBookingsCount: document.querySelector("#selected-bookings-count"),
  selectedBookingsList: document.querySelector("#selected-bookings-list"),
  bookingModal: document.querySelector("#booking-modal"),
  bookingModalContext: document.querySelector("#booking-modal-context"),
  bookingModalForm: document.querySelector("#booking-modal-form"),
  modalUserName: document.querySelector("#modal-user-name"),
  modalEndTime: document.querySelector("#modal-end-time"),
  modalNotes: document.querySelector("#modal-notes"),
  closeModal: document.querySelector("#close-modal"),
  cancelModal: document.querySelector("#cancel-modal"),
};

document.addEventListener("DOMContentLoaded", init);

async function init() {
  bindEvents();

  try {
    const initialData = await loadInitialData();
    state.data = normalizeData(initialData);
    state.activeWeekStart = getWeekStartFromIso(state.data.defaultWeek || getCurrentWeekValue());
    state.selectedStationId = state.data.stations[0]?.id || null;
    syncLayoutSettings();
    render();
    setFeedback("Tocca una piazzola nella vista settimanale e poi scegli uno slot libero.");
  } catch (error) {
    console.error(error);
    setFeedback("Impossibile caricare il file JSON iniziale.", true);
  }
}

function bindEvents() {
  elements.weekPicker.addEventListener("change", handleWeekChange);
  elements.overviewGrid.addEventListener("click", handleOverviewClick);
  elements.calendarGrid.addEventListener("click", handleCalendarClick);
  elements.selectedBookingsList.addEventListener("click", handleDeleteClick);
  elements.bookingModalForm.addEventListener("submit", handleModalSubmit);
  elements.closeModal.addEventListener("click", closeBookingModal);
  elements.cancelModal.addEventListener("click", closeBookingModal);
  elements.bookingModal.addEventListener("click", handleModalBackdropClick);
  document.addEventListener("keydown", handleKeydown);
}

async function loadInitialData() {
  const localData = window.localStorage.getItem(STORAGE_KEY);
  if (localData) {
    return JSON.parse(localData);
  }

  const response = await fetch(DATA_URL, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`Errore caricamento JSON: ${response.status}`);
  }

  return response.json();
}

function normalizeData(rawData) {
  const defaultColors = ["#1f7a8c", "#bf6c3b", "#4c956c", "#7a5c99"];

  return {
    version: rawData.version || 1,
    updatedAt: rawData.updatedAt || new Date().toISOString(),
    defaultWeek: rawData.defaultWeek || getCurrentWeekValue(),
    settings: {
      slotMinutes: Number(rawData.settings?.slotMinutes) || 30,
      startHour: Number(rawData.settings?.startHour) || 7,
      endHour: Number(rawData.settings?.endHour) || 18.5,
    },
    stations: Array.isArray(rawData.stations)
      ? rawData.stations.map((station, index) => ({
          ...station,
          name: sanitizeStationName(station.name || station.id || ""),
          color: station.color || defaultColors[index % defaultColors.length],
        }))
      : [],
    bookings: Array.isArray(rawData.bookings)
      ? rawData.bookings.map((booking) => ({
          id: booking.id || crypto.randomUUID(),
          stationId: booking.stationId,
          date: booking.date,
          startTime: booking.startTime,
          endTime: booking.endTime,
          userName: booking.userName,
          notes: booking.notes || "",
        }))
      : [],
  };
}

function handleWeekChange(event) {
  state.activeWeekStart = getWeekStartFromIso(event.target.value);
  render();
}

function handleOverviewClick(event) {
  const stationButton = event.target.closest("[data-station-id]");
  if (stationButton) {
    setSelectedStation(stationButton.dataset.stationId);
    return;
  }

  const cell = event.target.closest("[data-overview-station-id]");
  if (!cell) {
    return;
  }

  setSelectedStation(cell.dataset.overviewStationId);

  const dayIndex = Number(cell.dataset.dayIndex);
  const firstFree = findFirstFreeTime(state.selectedStationId, dayIndex);
  if (firstFree) {
    setFeedback(`Piazzola aggiornata. Primo slot libero: ${DAYS[dayIndex]} alle ${firstFree}.`);
  }
}

function handleCalendarClick(event) {
  const deleteButton = event.target.closest(".delete-booking");
  if (deleteButton) {
    deleteBooking(deleteButton.dataset.bookingId);
    return;
  }

  const slot = event.target.closest(".calendar-slot.is-free");
  if (!slot) {
    return;
  }

  openBookingModal(Number(slot.dataset.dayIndex), slot.dataset.time);
}

function handleDeleteClick(event) {
  const deleteButton = event.target.closest(".delete-booking");
  if (deleteButton) {
    deleteBooking(deleteButton.dataset.bookingId);
  }
}

function handleModalBackdropClick(event) {
  if (event.target.dataset.closeModal === "true") {
    closeBookingModal();
  }
}

function handleKeydown(event) {
  if (event.key === "Escape" && !elements.bookingModal.classList.contains("hidden")) {
    closeBookingModal();
  }
}

function openBookingModal(dayIndex, startTime) {
  const date = addDays(state.activeWeekStart, dayIndex);
  const dateKey = formatDateKey(date);
  const station = getSelectedStation();
  const allowedEndTimes = getAllowedEndTimes(station.id, dateKey, startTime);

  if (!allowedEndTimes.length) {
    setFeedback("Nessuna fascia libera disponibile per questo slot.", true);
    return;
  }

  state.pendingBooking = {
    stationId: station.id,
    dayIndex,
    dateKey,
    startTime,
  };

  elements.bookingModalContext.textContent = `${station.name} · ${DAYS[dayIndex]} ${formatDisplayDateShort(date)} · inizio ${startTime}`;
  elements.modalEndTime.innerHTML = allowedEndTimes
    .map((time) => `<option value="${time}">${time}</option>`)
    .join("");
  elements.modalEndTime.value = allowedEndTimes[0];
  elements.modalNotes.value = "";
  elements.bookingModal.classList.remove("hidden");
  elements.bookingModal.setAttribute("aria-hidden", "false");
  document.body.classList.add("is-modal-open");
  window.setTimeout(() => elements.modalUserName.focus(), 20);
}

function closeBookingModal() {
  state.pendingBooking = null;
  elements.bookingModal.classList.add("hidden");
  elements.bookingModal.setAttribute("aria-hidden", "true");
  elements.bookingModalForm.reset();
  document.body.classList.remove("is-modal-open");
}

function handleModalSubmit(event) {
  event.preventDefault();

  if (!state.pendingBooking) {
    return;
  }

  const userName = elements.modalUserName.value.trim();
  const endTime = elements.modalEndTime.value;
  const notes = elements.modalNotes.value.trim();

  const booking = {
    id: crypto.randomUUID(),
    stationId: state.pendingBooking.stationId,
    date: state.pendingBooking.dateKey,
    startTime: state.pendingBooking.startTime,
    endTime,
    userName,
    notes,
  };

  const validationError = validateBooking(booking);
  if (validationError) {
    setFeedback(validationError, true);
    return;
  }

  state.data.bookings.push(booking);
  persistData();
  closeBookingModal();
  render();
  setFeedback("Prenotazione registrata correttamente.");
}

function validateBooking(booking) {
  if (!booking.userName) {
    return "Il nome e obbligatorio.";
  }

  if (!booking.stationId || !booking.date || !booking.startTime || !booking.endTime) {
    return "Seleziona uno slot valido.";
  }

  if (timeToMinutes(booking.endTime) <= timeToMinutes(booking.startTime)) {
    return "L'orario di fine deve essere successivo all'orario di inizio.";
  }

  const overlap = state.data.bookings.find((existing) => {
    if (existing.stationId !== booking.stationId || existing.date !== booking.date) {
      return false;
    }

    return rangesOverlap(existing.startTime, existing.endTime, booking.startTime, booking.endTime);
  });

  if (overlap) {
    return `La piazzola selezionata e gia occupata dalle ${overlap.startTime} alle ${overlap.endTime}.`;
  }

  return "";
}

function deleteBooking(bookingId) {
  if (!window.confirm("Confermi la rimozione della prenotazione?")) {
    return;
  }

  state.data.bookings = state.data.bookings.filter((booking) => booking.id !== bookingId);
  persistData();
  render();
  setFeedback("Prenotazione rimossa.");
}

function render() {
  renderWeekRange();
  renderSummary();
  renderOverview();
  renderSelectedStation();
}

function renderWeekRange() {
  const weekEnd = addDays(state.activeWeekStart, DAYS.length - 1);
  elements.weekPicker.value = getWeekValueFromDate(state.activeWeekStart);
  elements.weekRange.textContent = `${formatDisplayDate(state.activeWeekStart)} - ${formatDisplayDate(weekEnd)}`;
}

function renderSummary() {
  const weekBookings = getBookingsForActiveWeek();
  elements.bookingCount.textContent = String(weekBookings.length);
}

function renderOverview() {
  const header = [
    '<div class="overview-head"><strong>Piazzola</strong></div>',
    ...DAYS.map((dayName, dayIndex) => {
      const date = addDays(state.activeWeekStart, dayIndex);
      return `
        <div class="overview-head">
          <strong>${dayName}</strong>
          <span>${formatDisplayDateShort(date)}</span>
        </div>
      `;
    }),
  ];

  const rows = state.data.stations.flatMap((station) => {
    const stationBookings = getBookingsForStationInActiveWeek(station.id);
    const label = `
      <button type="button" class="overview-row-label station-pill${station.id === state.selectedStationId ? " is-active" : ""}" data-station-id="${station.id}" style="--station-color:${station.color};">
        <strong><span class="station-color-dot"></span>${station.name}</strong>
      </button>
    `;

    const dayCells = DAYS.map((_, dayIndex) => {
      const dateKey = formatDateKey(addDays(state.activeWeekStart, dayIndex));
      const bookings = stationBookings
        .filter((booking) => booking.date === dateKey)
        .sort(compareBookings);
      const selectedClass = station.id === state.selectedStationId ? " is-selected" : "";

      return `
        <div class="overview-cell${selectedClass}" data-overview-station-id="${station.id}" data-day-index="${dayIndex}">
          ${renderOverviewDaySummary(station, bookings)}
        </div>
      `;
    });

    return [label, ...dayCells];
  });

  elements.overviewGrid.innerHTML = [...header, ...rows].join("");
}

function renderOverviewDaySummary(station, bookings) {
  if (!bookings.length) {
    return '<p class="overview-empty">Disponibile</p>';
  }

  const totalMinutes = bookings.reduce(
    (sum, booking) => sum + (timeToMinutes(booking.endTime) - timeToMinutes(booking.startTime)),
    0
  );
  const availableMinutes = getAvailableDayMinutes();
  const occupancyRate = Math.round((totalMinutes / availableMinutes) * 100);
  const bars = bookings
    .map((booking) => {
      const startOffset = timeToMinutes(booking.startTime) - state.data.settings.startHour * 60;
      const duration = timeToMinutes(booking.endTime) - timeToMinutes(booking.startTime);
      const left = (startOffset / availableMinutes) * 100;
      const width = (duration / availableMinutes) * 100;
      return `<span class="overview-mini-bar" style="left:${left}%;width:${Math.max(width, 4)}%;background:${station.color};"></span>`;
    })
    .join("");

  return `
    <article class="overview-day-summary">
      <div class="overview-day-stats">
        <strong>${bookings.length} pren.</strong>
        <span>${formatDurationLabel(totalMinutes)}</span>
      </div>
      <div class="overview-mini-track">${bars}</div>
      <p class="overview-day-caption">${occupancyRate}% occupato</p>
    </article>
  `;
}

function renderSelectedStation() {
  const station = getSelectedStation();
  if (!station) {
    return;
  }

  const stationBookings = getBookingsForStationInActiveWeek(station.id);
  elements.selectedStationName.textContent = station.name;
  elements.selectedStationMeta.textContent = "Tocca uno slot libero per inserire una prenotazione";
  renderCalendar(station, stationBookings);
  renderSelectedBookings(stationBookings);
}

function renderCalendar(station, stationBookings) {
  const times = getTimeOptions().slice(0, -1);
  const gridMarkup = [
    '<div class="calendar-corner">Ora</div>',
    ...DAYS.map((dayName, dayIndex) => {
      const date = addDays(state.activeWeekStart, dayIndex);
      return `
        <div class="calendar-day-head">
          <strong>${dayName}</strong>
          <span>${formatDisplayDateShort(date)}</span>
        </div>
      `;
    }),
  ];

  times.forEach((time) => {
    const halfHourClass = isHalfHourSlot(time) ? " is-half-hour" : "";
    gridMarkup.push(`<div class="calendar-time${halfHourClass}">${formatCalendarTimeLabel(time)}</div>`);

    DAYS.forEach((_, dayIndex) => {
      const dateKey = formatDateKey(addDays(state.activeWeekStart, dayIndex));
      const bookingStarting = stationBookings.find((booking) => booking.date === dateKey && booking.startTime === time);
      const isCovered = stationBookings.some((booking) => booking.date === dateKey && timeBelongsToBooking(time, booking));
      const slotClass = isHalfHourSlot(time) ? " is-half-hour" : "";

      if (bookingStarting) {
        const height = `calc(${getBookingSlotSpan(bookingStarting)} * var(--slot-height) - 4px)`;
        gridMarkup.push(`
          <div class="calendar-slot is-occupied${slotClass}" data-day-index="${dayIndex}" data-time="${time}">
            <article class="calendar-booking" style="background:${station.color}; height:${height};">
              <strong>${bookingStarting.startTime} - ${bookingStarting.endTime}</strong>
              <span>${bookingStarting.userName}</span>
              <small>${bookingStarting.notes || "Prenotazione attiva"}</small>
              <button type="button" class="delete-booking" aria-label="Rimuovi prenotazione" title="Rimuovi prenotazione" data-booking-id="${bookingStarting.id}">&times;</button>
            </article>
          </div>
        `);
      } else {
        gridMarkup.push(`<div class="calendar-slot ${isCovered ? "is-occupied" : "is-free"}${slotClass}" data-day-index="${dayIndex}" data-time="${time}"></div>`);
      }
    });
  });

  elements.calendarGrid.innerHTML = gridMarkup.join("");
}

function renderSelectedBookings(bookings) {
  const ordered = [...bookings].sort(compareBookings);
  elements.selectedBookingsCount.textContent = `${ordered.length} elementi`;
  elements.selectedBookingsList.innerHTML = ordered.length
    ? ordered.map((booking) => `
        <article class="selected-booking-card">
          <div class="selected-booking-head">
            <div>
              <strong>${getDayNameFromDateKey(booking.date)} · ${booking.startTime} - ${booking.endTime}</strong>
              <p>${booking.userName}</p>
            </div>
            <button type="button" class="delete-booking" aria-label="Rimuovi prenotazione" title="Rimuovi prenotazione" data-booking-id="${booking.id}">&times;</button>
          </div>
          <small>${booking.notes || "Nessuna nota"}</small>
        </article>
      `).join("")
    : '<p class="empty-list">Nessuna prenotazione per la piazzola selezionata in questa settimana.</p>';
}

function getAllowedEndTimes(stationId, dateKey, startTime) {
  const allTimes = getTimeOptions();
  const startMinutes = timeToMinutes(startTime);
  const candidateEndTimes = allTimes.filter((time) => timeToMinutes(time) > startMinutes);
  const allowed = [];

  candidateEndTimes.forEach((endTime) => {
    const overlaps = state.data.bookings.some((booking) => {
      if (booking.stationId !== stationId || booking.date !== dateKey) {
        return false;
      }

      return rangesOverlap(booking.startTime, booking.endTime, startTime, endTime);
    });

    if (!overlaps && !allowed.includes(endTime)) {
      allowed.push(endTime);
    }
  });

  return allowed;
}

function getAvailableDayMinutes() {
  return state.data.settings.endHour * 60 - state.data.settings.startHour * 60;
}

function getBookingsForActiveWeek() {
  const weekStartKey = formatDateKey(state.activeWeekStart);
  const weekEndKey = formatDateKey(addDays(state.activeWeekStart, DAYS.length - 1));
  return state.data.bookings.filter((booking) => booking.date >= weekStartKey && booking.date <= weekEndKey);
}

function getBookingsForStationInActiveWeek(stationId) {
  return getBookingsForActiveWeek().filter((booking) => booking.stationId === stationId);
}

function persistData() {
  state.data.updatedAt = new Date().toISOString();
  state.data.defaultWeek = getWeekValueFromDate(state.activeWeekStart);
  window.localStorage.setItem(STORAGE_KEY, JSON.stringify(state.data, null, 2));
}


function setFeedback(message, isError = false) {
  elements.feedback.textContent = message;
  elements.feedback.classList.toggle("is-error", isError);
}

function setSelectedStation(stationId) {
  state.selectedStationId = stationId;
  render();
}

function syncLayoutSettings() {
  document.documentElement.style.setProperty("--day-count", String(DAYS.length));
}

function getSelectedStation() {
  return state.data.stations.find((station) => station.id === state.selectedStationId) || null;
}

function findFirstFreeTime(stationId, dayIndex) {
  const dateKey = formatDateKey(addDays(state.activeWeekStart, dayIndex));
  const startTimes = getTimeOptions().slice(0, -1);
  return startTimes.find((time) => getAllowedEndTimes(stationId, dateKey, time).length > 0) || "";
}

function addDays(date, amount) {
  const nextDate = new Date(date);
  nextDate.setDate(nextDate.getDate() + amount);
  return nextDate;
}

function getCurrentWeekValue() {
  return getWeekValueFromDate(getWeekStart(new Date()));
}

function getWeekValueFromDate(date) {
  const isoParts = getIsoWeekParts(date);
  return `${isoParts.year}-W${String(isoParts.week).padStart(2, "0")}`;
}

function getWeekStart(date) {
  const nextDate = new Date(date);
  const day = nextDate.getDay() || 7;
  nextDate.setHours(0, 0, 0, 0);
  nextDate.setDate(nextDate.getDate() - day + 1);
  return nextDate;
}

function getWeekStartFromIso(value) {
  const [yearPart, weekPart] = value.split("-W");
  const year = Number(yearPart);
  const week = Number(weekPart);
  const januaryFourth = new Date(Date.UTC(year, 0, 4));
  const firstWeekStart = getWeekStart(januaryFourth);
  return addDays(firstWeekStart, (week - 1) * 7);
}

function getIsoWeekParts(date) {
  const target = new Date(date);
  target.setHours(0, 0, 0, 0);
  target.setDate(target.getDate() + 3 - ((target.getDay() + 6) % 7));
  const isoYear = target.getFullYear();
  const firstThursday = new Date(isoYear, 0, 4);
  firstThursday.setDate(firstThursday.getDate() + 3 - ((firstThursday.getDay() + 6) % 7));
  const week = 1 + Math.round((target - firstThursday) / 604800000);
  return { year: isoYear, week };
}

function minutesToTime(totalMinutes) {
  const hours = String(Math.floor(totalMinutes / 60)).padStart(2, "0");
  const minutes = String(totalMinutes % 60).padStart(2, "0");
  return `${hours}:${minutes}`;
}

function timeToMinutes(time) {
  const [hours, minutes] = time.split(":").map(Number);
  return hours * 60 + minutes;
}

function formatDurationLabel(totalMinutes) {
  const hours = Math.floor(totalMinutes / 60);
  const minutes = totalMinutes % 60;
  if (!minutes) {
    return `${hours}h`;
  }

  return `${hours}h ${String(minutes).padStart(2, "0")}`;
}

function sanitizeStationName(name) {
  return String(name).replace(/^Postazione\s+/i, "").trim();
}

function isHalfHourSlot(time) {
  return time.endsWith(":30");
}

function formatCalendarTimeLabel(time) {
  return isHalfHourSlot(time) ? "" : time;
}

function rangesOverlap(startA, endA, startB, endB) {
  return timeToMinutes(startB) < timeToMinutes(endA) && timeToMinutes(endB) > timeToMinutes(startA);
}

function getTimeOptions() {
  const options = [];
  const startMinutes = state.data.settings.startHour * 60;
  const endMinutes = state.data.settings.endHour * 60;
  const step = state.data.settings.slotMinutes;

  for (let minutes = startMinutes; minutes <= endMinutes; minutes += step) {
    options.push(minutesToTime(minutes));
  }

  return options;
}

function getBookingSlotSpan(booking) {
  return (timeToMinutes(booking.endTime) - timeToMinutes(booking.startTime)) / state.data.settings.slotMinutes;
}

function timeBelongsToBooking(time, booking) {
  const minutes = timeToMinutes(time);
  return minutes > timeToMinutes(booking.startTime) && minutes < timeToMinutes(booking.endTime);
}

function formatDateKey(date) {
  const year = date.getFullYear();
  const month = String(date.getMonth() + 1).padStart(2, "0");
  const day = String(date.getDate()).padStart(2, "0");
  return `${year}-${month}-${day}`;
}

function formatDisplayDate(date) {
  return new Intl.DateTimeFormat("it-IT", {
    day: "2-digit",
    month: "long",
    year: "numeric",
  }).format(date);
}

function formatDisplayDateShort(date) {
  return new Intl.DateTimeFormat("it-IT", {
    day: "2-digit",
    month: "short",
  }).format(date);
}

function getDayNameFromDateKey(dateKey) {
  const [year, month, day] = dateKey.split("-").map(Number);
  const date = new Date(year, month - 1, day);
  return DAYS[(date.getDay() + 6) % 7];
}

function compareBookings(left, right) {
  if (left.date !== right.date) {
    return left.date.localeCompare(right.date);
  }
  return timeToMinutes(left.startTime) - timeToMinutes(right.startTime);
}