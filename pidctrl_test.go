package pidctrl

import (
	"fmt"
	"testing"
	"time"
)

var tests = []struct {
	p       float64
	i       float64
	d       float64
	updates []*testUpdate
}{
	// p-only controller
	{
		p: 0.5,
		updates: []*testUpdate{
			{setpoint: 10, input: 5, output: 2.5},
			{input: 10, output: 0},
			{input: 15, output: -2.5},
			{input: 100, output: -45},
			{setpoint: 1, input: 0, output: 0.5},
		},
	},
	// i-only controller
	{
		i: 0.5,
		updates: []*testUpdate{
			{setpoint: 10, input: 5, duration: time.Second, output: 2.5},
			{input: 5, duration: time.Second, output: 5},
			{input: 5, duration: time.Second, output: 7.5},
			{input: 15, duration: time.Second, output: 5},
			{input: 20, duration: time.Second, output: 0},
		},
	},
	// d-only controller
	{
		d: 0.5,
		updates: []*testUpdate{
			{setpoint: 10, input: 5, duration: time.Second, output: 2.5},
			{input: 5, duration: time.Second, output: 0},
			{input: 10, duration: time.Second, output: -2.5},
		},
	},
	// pid controller
	{
		p: 0.5,
		i: 0.5,
		d: 0.5,
		updates: []*testUpdate{
			{setpoint: 10, input: 5, duration: time.Second, output: 7.5},
			{input: 10, duration: time.Second, output: 0},
			{input: 15, duration: time.Second, output: -5},
			{input: 100, duration: time.Second, output: -132.5},
			{setpoint: 1, duration: time.Second, input: 0, output: 1.5},
		},
	},
}

type testUpdate struct {
	setpoint float64
	input    float64
	duration time.Duration
	output   float64
}

func (u *testUpdate) check(c *PIDController) error {
	if u.setpoint != 0 {
		c.Set(u.setpoint)
	}
	output := c.UpdateDuration(u.input, u.duration)
	if output != u.output {
		return fmt.Errorf("Bad output: %f != %f (%#v)", output, u.output, u)
	}
	return nil
}

func TestUpdate_p(t *testing.T) {
	for i, test := range tests {
		t.Logf("-- test #%d", i+1)
		c := NewPIDController(test.p, test.i, test.d)
		for _, u := range test.updates {
			if err := u.check(c); err != nil {
				t.Error(err)
			}
		}
	}
}
