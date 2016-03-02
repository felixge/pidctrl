// Packege pidctrl implements a PID controller.
//
// see http://en.wikipedia.org/wiki/PID_controller
package pidctrl

import (
	"fmt"
	"math"
	"time"
)

type MinMaxError struct {
	min, max float64
}

func (e MinMaxError) Error() string {
	return fmt.Sprintf("min: %v is greater than max: %v", e.min, e.max)
}

// NewPIDController returns a new PIDController using the given gain values.
func NewPIDController(p, i, d float64) *PIDController {
	return &PIDController{p: p, i: i, d: d, outMin: math.Inf(-1), outMax: math.Inf(0)}
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
	outMin     float64   // Output Min
	outMax     float64   // Output Max
}

// Set changes the setpoint of the controller.
func (c *PIDController) Set(setpoint float64) *PIDController {
	c.setpoint = setpoint
	return c
}

// Get returns the setpoint of the controller.
func (c *PIDController) Get() float64 {
	return c.setpoint
}

// SetPID changes the P, I, and D constants
func (c *PIDController) SetPID(p, i, d float64) *PIDController {
	c.p = p
	c.i = i
	c.d = d
	return c
}

// PID returns the P, I, and D constants
func (c *PIDController) PID() (p, i, d float64) {
	return c.p, c.i, c.d
}

// SetOutputLimits sets the min and max output values
func (c *PIDController) SetOutputLimits(min, max float64) *PIDController {
	if min > max {
		panic(MinMaxError{min, max})
	}
	c.outMin = min
	c.outMax = max

	if c.integral > c.outMax {
		c.integral = c.outMax
	} else if c.integral < c.outMin {
		c.integral = c.outMin
	}
	return c
}

// OutputLimits returns the min and max output values
func (c *PIDController) OutputLimits() (min, max float64) {
	return c.outMin, c.outMax
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
	if c.integral > c.outMax {
		c.integral = c.outMax
	} else if c.integral < c.outMin {
		c.integral = c.outMin
	}
	if dt > 0 {
		d = -((value - c.prevValue) / dt)
	}
	c.prevValue = value
	output := (c.p * err) + c.integral + (c.d * d)

	if output > c.outMax {
		output = c.outMax
	} else if output < c.outMin {
		output = c.outMin
	}

	return output
}
