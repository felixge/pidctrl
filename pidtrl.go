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
	prevErr    float64   // error from last update
	integral   float64   // integral sum
	lastUpdate time.Time // time of last update
}

// Set changes the setpoint of the controller.
func (c *PIDController) Set(setpoint float64) {
	c.setpoint = setpoint
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
	c.integral += err * dt
	if dt > 0 {
		d = (err - c.prevErr) / dt
	}
	c.prevErr = err
	return (c.p * err) + (c.i * c.integral) + (c.d * d)
}
