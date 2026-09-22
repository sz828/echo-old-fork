#pragma once

/**
 * Elim to define different auton routines
 */
enum Auton_ {
  SKILLS,
  NONE
} typedef Auton;

/**
 * Defines which alliance we are on
 */
enum Alliance_ {
  RED = 1,
  BLUE = 2
} typedef Alliance;

#define OPPONENTS (ALLIANCE == RED ? BLUE : RED)

#define AUTON NONE
inline auto ALLIANCE=RED;
