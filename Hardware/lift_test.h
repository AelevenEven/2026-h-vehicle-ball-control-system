#ifndef LIFT_TEST_H
#define LIFT_TEST_H

/*
 * Stand-alone X42S lift bring-up mode.
 *
 * This mode deliberately does not depend on K230 BALL frames.  It consumes
 * the existing board key, keeps both chassis motors stopped, and moves the
 * X42S through a small, guarded angle sequence for mechanical verification.
 */
void LiftTest_Init(void);
void LiftTest_Update(void);

#endif /* LIFT_TEST_H */
