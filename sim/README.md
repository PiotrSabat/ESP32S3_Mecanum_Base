# Symulatory — testowanie sterowania bez sprzętu

Te programy uruchamiają **prawdziwy kod z `src/`** (`Motor.cpp`, `MecanumDrive.cpp`)
na komputerze, podstawiając zegar, enkoder i wyjścia PWM. Dzięki temu można
sprawdzić zachowanie regulatora, którego na jeżdżącej platformie nie da się
podejrzeć — stany wewnętrzne, przebiegi w czasie, prąd pobierany przez silniki.

To nie zastępuje próby na podłodze. Kompilacja i zielone symulacje dowodzą
tylko, że logika robi to, co zamierzono; czy platforma faktycznie jedzie tam,
gdzie wskazuje drążek, rozstrzyga wyłącznie sprzęt.

## Uruchomienie

```bash
./sim/run.sh              # wszystkie symulacje
./sim/run.sh failsafe     # jedna wybrana
```

Potrzebny tylko `g++`. Wyniki lądują w `sim/build/` (katalog ignorowany przez git).

## Co sprawdza która

| Plik | Zakres |
|---|---|
| `failsafe.cpp` | wykrycie utraty łączności z padem, odcięcie napędu, powrót sterowania, przepełnienie `millis()` po ~49,7 dnia |
| `kinematyka.cpp` | przeliczenie drążka na RPM, martwa strefa, normalizacja czterech kół (czy kierunek jazdy przeżywa nasycenie) |
| `hamowanie.cpp` | prąd przy rozruchu i hamowaniu, wyrzut w rewers po zatrzymaniu |
| `trzymanie.cpp` | opór przy ręcznym obracaniu koła oraz prąd hamowania z pełnej prędkości |

## Model silnika — czytaj to, zanim uwierzysz w liczby

Parametry napędu są **oszacowane**, nie zmierzone: rezystancja uzwojenia
wyliczona z zakładanego prądu zwarciowego 1,5 A przy 6 V, stała czasowa 150 ms,
strefa martwa 25 jednostek PWM. Prąd liczony jest w dwóch wariantach
(pesymistycznym i uwzględniającym wybieg w fazie wyłączenia), bo rzeczywistość
zależy od trybu rozładowania mostka MX1508.

Dlatego wartości bezwzględne traktuj jako rząd wielkości, a **porównania między
wariantami** — jako wiarygodne. Dokładnie tak powstały dzisiejsze decyzje:
liczyło się, że jedno rozwiązanie ciągnie trzykrotnie mniej prądu niż drugie,
a nie ile dokładnie amperów.

Historia z 2026-08-23 pokazuje granice tej metody: symulator dobrał nastawy PID
poprawnie pod względem prędkości, ale nie modelował wtedy prądu — i wgrany
firmware wywalał zabezpieczenie ogniw. Model odpowiada tylko na pytanie, które
mu się zada.
