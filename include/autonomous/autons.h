#pragma once

/**
 * Autonomous routines. AUTON in auton.h picks one at compile time.
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
