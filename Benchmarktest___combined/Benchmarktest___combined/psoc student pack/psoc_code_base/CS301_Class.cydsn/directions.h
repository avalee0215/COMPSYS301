#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Call once at boot (optional). */
void Directions_Init(void);

/* Returns true while a timed pivot is running (this impl is blocking, so false). */
bool Directions_IsBusy(void);

/* If *gdir == 1 -> do left turn; if *gdir == 2 -> do right turn.
 * When done, sets *gdir = 0.  (Blocking ~0.6–0.8 s; tune constants in .c)
 */
void Directions_Handle(volatile uint8_t* gdir);
