// Packege pidctrl implements a PID controller.
//
// see http://en.wikipedia.org/wiki/PID_controller
package pidctrl

import (
	"time"
)

// NewPIDController returns a new PIDController using the given gain values.
func NewPIDController(p, i, d float64) *PIDController {
	return &PIDController{p: p, i: i, d: d}
}

// PIDController implements a PID controller.
type PIDController struct {
	p          float64   // proportional gain
	i          float64   // integral gain
	d          float64   // derrivate gain
	setpoint   float64   // current setpoint
	prevValue  float64   // last process value
	integral   float64   // integral sum
	lastUpdate time.Time // time of last update
}

// Set changes the setpoint of the controller.
func (c *PIDController) Set(setpoint float64) {
	c.setpoint = setpoint
}

// Get returns the setpoint of the controller.
func (c *PIDController) Get() float64 {
	return c.setpoint
}

// SetPID changes the P, I, and D constants
func (c *PIDController) SetPID(p, i, d float64) {
	c.p = p
	c.i = i
	c.d = d
}

// GetPID returns the P, I, and D constants
func (c *PIDController) GetPID() (p, i, d float64) {
	return c.p, c.i, c.d
}

// Update is identical to UpdateDuration, but automatically keeps track of the
// durations between updates.
func (c *PIDController) Update(value float64) float64 {
	var duration time.Duration
	if !c.lastUpdate.IsZero() {
		duration = time.Since(c.lastUpdate)
	}
	c.lastUpdate = time.Now()
	return c.UpdateDuration(value, duration)
}

// UpdateDuration updates the controller with the given value and duration since
// the last update. It returns the new output.
//
// see http://en.wikipedia.org/wiki/PID_controller#Pseudocode
func (c *PIDController) UpdateDuration(value float64, duration time.Duration) float64 {
	var (
		dt  = duration.Seconds()
		err = c.setpoint - value
		d   float64
	)
	c.integral += err * dt * c.i
	if dt > 0 {
		d = -((value - c.prevValue) / dt)
	}
	c.prevValue = value
	return (c.p * err) + c.integral + (c.d * d)
}
