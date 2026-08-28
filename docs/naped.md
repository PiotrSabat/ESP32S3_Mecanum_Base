# Napęd — dane do przeliczeń

Wartości potrzebne, żeby z obrotów zrobić prędkość, i żeby nie zgadywać
przy kolejnych zmianach. Uzupełniane w miarę pomiarów.

## Koła

| wielkość | wartość | źródło |
|---|---|---|
| średnica koła mecanum | **60 mm** | Piotr, 2026-08-27 |
| obwód | 188,5 mm | π × 60 mm |
| `DEFAULT_GEAR_RATIO` | 960 impulsów na obrót **koła** | `parameters.h` |
| przełożenie | 120:1 | odwracalne (koło da się obrócić ręką) |

## Geometria podwozia

Mierzone **od środka koła do środka koła** (Piotr, 2026-08-28, szkic na kratce).

| wielkość | wartość |
|---|---|
| rozstaw osi (przód–tył) | **12 cm** |
| rozstaw kół (lewo–prawo) | **15 cm** |
| szerokość koła | prawie 3 cm |

Rozstaw lewo–prawo bywa mylący: prześwit między kołami to 12 cm, ale koła są
szerokie, więc od środka do środka wychodzi 15 cm. **Do kinematyki liczy się
odległość między środkami**, bo tam działa siła.

## Przeliczenie obrotów na prędkość

    v [m/s] = RPM × π × D / 60 = RPM × 0,0031416   (dla D = 60 mm)
    v [km/h] = v [m/s] × 3,6

Punkty odniesienia:

| RPM koła | m/s | km/h |
|---|---|---|
| 30 | 0,094 | 0,34 |
| 90 | 0,283 | 1,02 |
| 180 (`MAX_RPM`) | 0,565 | 2,03 |

**Uwaga:** `MAX_RPM = 180` jest wartością **założoną**, nie zmierzoną. Odczyt
prędkości liczony ze *zmierzonych* obrotów pozwoli to wreszcie sprawdzić:
jeśli przy pełnym wychyleniu drążka obroty wypłaszczają się poniżej 180,
wartość jest za wysoka i skalowanie drążka marnuje ostatni kawałek skoku.

## Wektor ruchu platformy z obrotów kół

Odwrócenie mieszania mecanum (`MecanumDrive::drive`) daje z czterech kół
prędkości platformy:

    vy = (FL + FR + RL + RR) / 4      do przodu
    vx = (FL − FR − RL + RR) / 4      w bok
    ω  = (FL − FR + RL − RR) / 4      obrót

Wynik jest w RPM i przelicza się na m/s tym samym wzorem co wyżej.

### Obrót w stopniach na sekundę

Człon obrotu z `mecanumInverse()` jest w jednostkach obrotów koła. Prędkość
liniowa, jaką koło musi mieć, żeby platforma kręciła się z prędkością kątową ω,
wynosi `ω × (lx + ly)`, gdzie `lx` i `ly` to **połowy** rozstawów:

    lx + ly = (0,12 + 0,15) / 2 = 0,135 m

Stąd `RPM01_TO_DEG_S` w `messages.h`. Przy pełnym wychyleniu obrotu wychodzi
**około 240 °/s**, czyli dwie trzecie obrotu na sekundę.

**Ograniczenie:** prędkość **boczna** wychodzi optymistyczna. Rolki mecanum
ślizgają się bocznie z zasady działania, więc rzeczywiste `vx` jest mniejsze
od policzonego. Przy jeździe wprost rozjazd jest niewielki, przy jeździe
bokiem — zauważalny. To nie jest błąd pomiaru, tylko fizyka napędu.
