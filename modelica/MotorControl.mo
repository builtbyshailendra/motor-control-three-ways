within ;
package MotorControl "One DC-motor speed loop (CTMS example) as a physical model"
  extends Modelica.Icons.Package;

  model Motor "Armature circuit, EMF, rotor inertia and friction: voltage in, speed out"
    extends Modelica.Blocks.Icons.Block;
    parameter Modelica.Units.SI.Resistance R = 1 "Armature resistance";
    parameter Modelica.Units.SI.Inductance L = 0.5 "Armature inductance";
    parameter Modelica.Units.SI.ElectricalTorqueConstant K = 0.01 "Torque constant = back-EMF constant";
    parameter Modelica.Units.SI.Inertia J = 0.01 "Rotor inertia";
    parameter Modelica.Units.SI.RotationalDampingConstant b = 0.1 "Viscous friction";
    Modelica.Blocks.Interfaces.RealInput v(unit = "V") "Armature voltage"
      annotation(Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
    Modelica.Blocks.Interfaces.RealOutput w(unit = "rad/s") "Rotor speed"
      annotation(Placement(transformation(extent = {{100, -10}, {120, 10}})));
    Modelica.Electrical.Analog.Sources.SignalVoltage source;
    Modelica.Electrical.Analog.Basic.Resistor resistor(R = R);
    Modelica.Electrical.Analog.Basic.Inductor inductor(L = L, i(start = 0, fixed = true));
    Modelica.Electrical.Analog.Basic.RotationalEMF emf(k = K);
    Modelica.Electrical.Analog.Basic.Ground ground;
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = J,
      phi(start = 0, fixed = true), w(start = 0, fixed = true));
    Modelica.Mechanics.Rotational.Components.Damper damper(d = b);
    Modelica.Mechanics.Rotational.Components.Fixed fixed;
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor;
  equation
    connect(v, source.v);
    connect(source.p, resistor.p);
    connect(resistor.n, inductor.p);
    connect(inductor.n, emf.p);
    connect(emf.n, source.n);
    connect(source.n, ground.p);
    connect(emf.flange, inertia.flange_a);
    connect(inertia.flange_b, damper.flange_a);
    connect(damper.flange_b, fixed.flange);
    connect(inertia.flange_b, speedSensor.flange);
    connect(speedSensor.w, w);
  end Motor;

  model SpeedLoop "Linear PI loop, 1 rad/s setpoint step, no supply limit"
    parameter Real kp = 20 "Proportional gain, V per rad/s";
    parameter Real ki = 40 "Integral gain, V per rad";
    parameter Modelica.Units.SI.AngularVelocity wRef = 1 "Setpoint step";
    Modelica.Blocks.Sources.Step setpoint(height = wRef, startTime = 0);
    Modelica.Blocks.Math.Feedback error;
    Modelica.Blocks.Continuous.PI pi(k = kp, T = kp / ki,
      initType = Modelica.Blocks.Types.Init.InitialState, x_start = 0);
    Motor motor;
  equation
    connect(setpoint.y, error.u1);
    connect(motor.w, error.u2);
    connect(error.y, pi.u);
    connect(pi.y, motor.v);
    annotation(experiment(StartTime = 0, StopTime = 3, Interval = 0.001, Tolerance = 1e-08));
  end SpeedLoop;

  model SpeedLoopSaturated "PI, then a +/-12 V supply limit; nothing stops the integrator winding up"
    parameter Real kp = 20 "Proportional gain, V per rad/s";
    parameter Real ki = 40 "Integral gain, V per rad";
    parameter Modelica.Units.SI.AngularVelocity wRef = 1 "Setpoint step";
    parameter Modelica.Units.SI.Voltage vMax = 12 "Supply limit";
    Modelica.Blocks.Sources.Step setpoint(height = wRef, startTime = 0);
    Modelica.Blocks.Math.Feedback error;
    Modelica.Blocks.Continuous.PI pi(k = kp, T = kp / ki,
      initType = Modelica.Blocks.Types.Init.InitialState, x_start = 0);
    Modelica.Blocks.Nonlinear.Limiter supply(uMax = vMax, uMin = -vMax);
    Motor motor;
  equation
    connect(setpoint.y, error.u1);
    connect(motor.w, error.u2);
    connect(error.y, pi.u);
    connect(pi.y, supply.u);
    connect(supply.y, motor.v);
    annotation(experiment(StartTime = 0, StopTime = 5, Interval = 0.001, Tolerance = 1e-08));
  end SpeedLoopSaturated;

  model SpeedLoopAntiWindup "LimPID as a PI with its built-in anti-windup (back-calculation)"
    parameter Real kp = 20 "Proportional gain, V per rad/s";
    parameter Real ki = 40 "Integral gain, V per rad";
    parameter Modelica.Units.SI.AngularVelocity wRef = 1 "Setpoint step";
    parameter Modelica.Units.SI.Voltage vMax = 12 "Supply limit";
    parameter Real Ni = 0.9 "Tracking time constant is Ni*Ti (LimPID's default 0.9)";
    Modelica.Blocks.Sources.Step setpoint(height = wRef, startTime = 0);
    Modelica.Blocks.Continuous.LimPID pid(
      controllerType = Modelica.Blocks.Types.SimpleController.PI,
      k = kp, Ti = kp / ki, yMax = vMax, yMin = -vMax, Ni = Ni,
      initType = Modelica.Blocks.Types.Init.InitialState, xi_start = 0);
    Motor motor;
  equation
    connect(setpoint.y, pid.u_s);
    connect(motor.w, pid.u_m);
    connect(pid.y, motor.v);
    annotation(experiment(StartTime = 0, StopTime = 5, Interval = 0.001, Tolerance = 1e-08));
  end SpeedLoopAntiWindup;

  model SpeedLoopLimPIDNoTracking "The same LimPID with tracking made negligible: should wind up like SpeedLoopSaturated"
    extends SpeedLoopAntiWindup(Ni = 1e9);
  end SpeedLoopLimPIDNoTracking;
end MotorControl;
