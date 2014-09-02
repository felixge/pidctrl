// Packege pidctrl implements a PID controller.
//
// see http://en.wikipedia.org/wiki/PID_controller
package pidctrl

import (
	"time"
)

type DeriveMethodType int

const (
	DeriveOnErr DeriveMethodType = iota
	DeriveOnValue
)

// NewPIDController returns a new PIDController using the given gain values.
func NewPIDController(p, i, d float64) *PIDController {
	return &PIDController{p: p, i: i, d: d}
}

// PIDController implements a PID controller.
type PIDController struct {
	p            float64          // proportional gain
	i            float64          // integral gain
	d            float64          // derrivate gain
	setpoint     float64          // current setpoint
	prevSetpoint float64          // last setpoint
	prevValue    float64          // last process value
	prevErr      float64          // error from last update
	integral     float64          // integral sum
	lastUpdate   time.Time        // time of last update
	deriveOn     DeriveMethodType // What do we derive on?
}

// Set changes the setpoint of the controller.
func (c *PIDController) Set(setpoint float64) {
	c.setpoint = setpoint
}

// Get returns the setpoint of the controller.
func (c *PIDController) Get() float64 {
	return c.setpoint
}

// SetDeriveMethod changes the derivation method.
func (c *PIDController) SetDeriveMethod(dm DeriveMethodType) {
	c.deriveOn = dm
}

// GetDeriveMethod returns the derivation method.
func (c *PIDController) GetDeriveMethod() DeriveMethodType {
	return c.deriveOn
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
		if c.deriveOn == DeriveOnErr {
			d = (err - c.prevErr) / dt
		} else {
			d = ((c.setpoint - c.prevSetpoint) / dt) - ((value - c.prevValue) / dt)
		}
	}
	c.prevValue = value
	c.prevSetpoint = c.setpoint
	c.prevErr = err
	return (c.p * err) + (c.i * c.integral) + (c.d * d)
}
