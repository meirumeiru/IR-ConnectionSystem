﻿using System;
using System.Collections;
using System.Collections.Generic;

using UnityEngine;

#if DEBUG
using IR_ConnectionSystem.Utility;
#endif
using DockingFunctions;


namespace IR_ConnectionSystem.Module
{
	public class ModuleIRBM : PartModule, IDockable, ITargetable, IModuleInfo
	{
		// Settings

		[KSPField(isPersistant = false), SerializeField]
		public string nodeTransformName = "dockingNode";

		[KSPField(isPersistant = false), SerializeField]
		public string referenceAttachNode = ""; // if something is connected to this node, then the state is "Attached" (or "Pre-Attached" -> connected in the VAB/SPH)

		[KSPField(isPersistant = false), SerializeField]
		public Vector3 dockingOrientation = Vector3.up; // defines the direction of the docking port (when docked at a 0° angle, these local vectors of two ports point into the same direction)

		[KSPField(isPersistant = false), SerializeField]
		public int snapCount = 1;


		[KSPField(isPersistant = false), SerializeField]
		public float detectionDistance = 5f;

		[KSPField(isPersistant = false), SerializeField]
		public float approachingDistance = 0.3f;

		[KSPField(isPersistant = false), SerializeField]
		public float approachingAngle = 15f;

		[KSPField(isPersistant = false), SerializeField]
		public float captureDistance = 0.03f;

		[KSPField(isPersistant = false), SerializeField]
		public float captureAngle = 5f;


		[KSPField(isPersistant = false), SerializeField]
		public string nodeType = "BM";

		[KSPField(isPersistant = false), SerializeField]
		private string nodeTypesAccepted = "BM";

		public HashSet<string> nodeTypesAcceptedS = null;

		[KSPField(isPersistant = false), SerializeField]
		public int supportedModes = 3; // passive = 1, active = 2, both = 3

		[KSPField(isPersistant = false), SerializeField]
		public bool autoActivePassive = false;

		[KSPField(isPersistant = false), SerializeField]
		public bool directUndock = false; // if true, ports don't go to latched after undocking but directly to released


		[KSPField(isPersistant = false), SerializeField]
		public float breakingForce = 100f;

		[KSPField(isPersistant = false), SerializeField]
		public float breakingTorque = 100f;

		[KSPField(isPersistant = false), SerializeField]
		public float latchingSpeedRotation = 0.1f; // degrees per second

		[KSPField(isPersistant = false), SerializeField]
		public float latchingSpeedTranslation = 0.025f; // distance per second


		[KSPField(isPersistant = false), SerializeField]
		public bool canCrossfeed = true;

		[KSPField(isPersistant = true)]
		public bool crossfeed = true;


		[KSPField(guiFormat = "S", guiActive = true, guiActiveEditor = true, guiName = "Port Name")]
		public string portName = "";

		// Docking and Status

		public BaseEvent evtSetAsTarget;
		public BaseEvent evtUnsetTarget;

		public Transform nodeTransform;

		public KerbalFSM fsm;

		public KFSMState st_active;			// "active" / "searching"
		public KFSMState st_passive;		// "passive"

		public KFSMState st_approaching;	// port found
		public KFSMState st_approaching_passive;

		public KFSMState st_latching;		// orienting and retracting in progress
		public KFSMState st_prelatched;		// ready to dock

		public KFSMState st_latched;		// docked
		public KFSMState st_latched_passive;

		public KFSMState st_docked;			// docked or docked_to_same_vessel
		public KFSMState st_preattached;

		public KFSMState st_disabled;


		public KFSMEvent on_setactive;
		public KFSMEvent on_setpassive;

		public KFSMEvent on_approach;
		public KFSMEvent on_approach_passive;

		public KFSMEvent on_distance;
		public KFSMEvent on_distance_passive;

		public KFSMEvent on_latching;
		public KFSMEvent on_prelatched;

		public KFSMEvent on_latch;
		public KFSMEvent on_latch_passive;

		public KFSMEvent on_release;
		public KFSMEvent on_release_passive;

		public KFSMEvent on_dock;
		public KFSMEvent on_dock_passive;

		public KFSMEvent on_undock;
		public KFSMEvent on_undock_passive;

		public KFSMEvent on_disable;

		public KFSMEvent on_construction;

		// Sounds
/* FEHLER, future
		[KSPField(isPersistant = false)] public string preAttachSoundFilePath = "";
		[KSPField(isPersistant = false)] public string latchSoundFilePath = "";
		[KSPField(isPersistant = false)] public string detachSoundFilePath = "";
		
		[KSPField(isPersistant = false)] public string activatingSoundFilePath = "";
		[KSPField(isPersistant = false)] public string activatedSoundFilePath = "";
		[KSPField(isPersistant = false)] public string deactivatingSoundFilePath = "";

		protected SoundSource soundSound = null;
*/
		// Capturing / Docking

		public ModuleIRBM otherPort;
		public uint dockedPartUId;
		public uint dockedType; // defines which side this part is -> 0 = part, 1 = targetPart

		public DockedVesselInfo vesselInfo;

		private bool inCaptureDistance = false;

		private ConfigurableJoint LatchJoint;

		private Quaternion LatchJointTargetRotation;
		private Vector3 LatchJointTargetPosition;

		private Vector3 LatchJointInitialPosition;

		private float latchProgress;
		private float latchProgressStep = 0.0005f;

		private int latchRelaxCounter;

		// Packed / OnRails

		private bool followOtherPort = false;

		private Vector3 otherPortRelativePosition;
		private Quaternion otherPortRelativeRotation;

		////////////////////////////////////////
		// Constructor

		public ModuleIRBM()
		{
		}

		////////////////////////////////////////
		// Callbacks

		public override void OnAwake()
		{
#if DEBUG
	//		DebugInit();
#endif

			part.dockingPorts.AddUnique(this);
		}

		public override void OnLoad(ConfigNode node)
		{
			base.OnLoad(node);

			if(node.HasValue("state"))
				DockStatus = node.GetValue("state");
			else
				DockStatus = "Inactive";

			if(node.HasValue("dockUId"))
				dockedPartUId = uint.Parse(node.GetValue("dockUId"));

			if(node.HasValue("dockedType"))
				dockedType = uint.Parse(node.GetValue("dockedType"));

			if(node.HasNode("DOCKEDVESSEL"))
			{
				vesselInfo = new DockedVesselInfo();
				vesselInfo.Load(node.GetNode("DOCKEDVESSEL"));
			}

			if(node.HasValue("followOtherPort"))
			{
				followOtherPort = bool.Parse(node.GetValue("followOtherPort"));

				node.TryGetValue("otherPortRelativePosition", ref otherPortRelativePosition);
				node.TryGetValue("otherPortRelativeRotation", ref otherPortRelativeRotation);
			}
		}

		public override void OnSave(ConfigNode node)
		{
			base.OnSave(node);

			node.AddValue("state", (string)(((fsm != null) && (fsm.Started)) ? fsm.currentStateName : DockStatus));

			node.AddValue("dockUId", dockedPartUId);

			node.AddValue("dockedType", dockedType);

			if(vesselInfo != null)
				vesselInfo.Save(node.AddNode("DOCKEDVESSEL"));

			node.AddValue("followOtherPort", followOtherPort);

			if(followOtherPort)
			{
				if(otherPortRelativePosition != null)	node.AddValue("otherPortRelativePosition", otherPortRelativePosition);
				if(otherPortRelativeRotation != null)	node.AddValue("otherPortRelativeRotation", otherPortRelativeRotation);
			}
		}

		public override void OnStart(StartState state)
		{
			base.OnStart(state);

			nodeTypesAcceptedS = new HashSet<string>();

			string[] values = nodeTypesAccepted.Split(new char[2] { ',', ' ' }, StringSplitOptions.RemoveEmptyEntries);
			foreach(string s in values)
				nodeTypesAcceptedS.Add(s);

			if(portName == string.Empty)
				portName = part.partInfo.title;

			if(state == StartState.Editor)
				return;

			evtSetAsTarget = base.Events["SetAsTarget"];
			evtUnsetTarget = base.Events["UnsetTarget"];

			GameEvents.onVesselGoOnRails.Add(OnPack);
			GameEvents.onVesselGoOffRails.Add(OnUnpack);

			GameEvents.OnEVAConstructionModePartDetached.Add(OnEVAConstructionModePartDetached);

			nodeTransform = part.FindModelTransform(nodeTransformName);
			if(!nodeTransform)
			{
				Logger.Log("No node transform found with name " + nodeTransformName, Logger.Level.Error);
				return;
			}

			StartCoroutine(WaitAndInitialize(state));

	//		StartCoroutine(WaitAndDisableDockingNode());
		}

		public IEnumerator WaitAndInitialize(StartState st)
		{
			yield return null;

			Events["TogglePort"].active = false;
			Events["ToggleMode"].active = false;

			Events["Latch"].active = false;
			Events["Release"].active = false;

			Events["Dock"].active = false;
			Events["Undock"].active = false;

			if(!canCrossfeed) crossfeed = false;

			part.fuelCrossFeed = crossfeed;

			Events["EnableXFeed"].active = !crossfeed;
			Events["DisableXFeed"].active = crossfeed;

			if(dockedPartUId != 0)
			{
				Part otherPart;

				while(!(otherPart = FlightGlobals.FindPartByID(dockedPartUId)))
					yield return null;

				otherPort = otherPart.GetComponent<ModuleIRBM>();

				// other port will load this too
				otherPort.otherPort = this;
				otherPort.dockedPartUId = part.flightID;
			}

			if((DockStatus == "Inactive")
			|| ((DockStatus == "Attached") && (otherPort == null))) // fix damaged state (just in case)
			{
// FEHLER, Frage: warum kann das passieren, dass hier "Attached" und null ist? hmm? woher kommt das?

				// fix state if attached to other port

				if(referenceAttachNode != string.Empty)
				{
					AttachNode node = part.FindAttachNode(referenceAttachNode);
					if((node != null) && node.attachedPart)
					{
						ModuleIRBM _otherPort = node.attachedPart.GetComponent<ModuleIRBM>();

						if(_otherPort)
						{
							otherPort = _otherPort;
							dockedPartUId = otherPort.part.flightID;

							DockStatus = "Attached";
							otherPort.DockStatus = "Attached";
						}
					}
				}
			}

			SetupFSM();

			if((DockStatus == "Approaching")
			|| (DockStatus == "Latching")
			|| (DockStatus == "Pre Latched")
			|| (DockStatus == "Latched"))
			{
				if(otherPort != null)
				{
					while(!otherPort.part.started || (otherPort.fsm == null) || (!otherPort.fsm.Started))
						yield return null;
				}
			}

			if(DockStatus == "Pre Latched")
				DockStatus = "Latching";

			if(DockStatus == "Latched")
			{
				BuildLatchJoint(otherPort);
				CalculateLatchJointTarget();
			}
			
			if(DockStatus == "Docked")
			{
				otherPort.DockStatus = "Docked";

				if(dockedType == 0)
					DockingHelper.OnLoad(this, vesselInfo, otherPort, otherPort.vesselInfo);
			}

			fsm.StartFSM(DockStatus);
		}
	/*
		public IEnumerator WaitAndDisableDockingNode()
		{
			ModuleDockingNode DockingNode = part.FindModuleImplementing<ModuleDockingNode>();

			if(DockingNode)
			{
				while((DockingNode.fsm == null) || (!DockingNode.fsm.Started))
					yield return null;

				DockingNode.fsm.RunEvent(DockingNode.on_disable);
			}
		}
	*/
		public void OnDestroy()
		{
			GameEvents.onVesselGoOnRails.Remove(OnPack);
			GameEvents.onVesselGoOffRails.Remove(OnUnpack);

			GameEvents.OnEVAConstructionModePartDetached.Remove(OnEVAConstructionModePartDetached);
		}

		private void OnPack(Vessel v)
		{
			if(vessel == v)
			{
				if((DockStatus == "Latched")
				|| (DockStatus == "Latched (passive)"))
				{
					if(Vessel.GetDominantVessel(vessel, otherPort.vessel) == otherPort.vessel)
					{
						followOtherPort = true;
						VesselPositionManager.Register(part, otherPort.part, true, out otherPortRelativePosition, out otherPortRelativeRotation);
					}
				}
			}
		}

		private void OnUnpack(Vessel v)
		{
			if(vessel == v)
			{
				if((DockStatus == "Latched")
				|| (DockStatus == "Latched (passive)"))
				{
					VesselPositionManager.Unregister(vessel);
					followOtherPort = false;
				}
			}
		}

		private void OnEVAConstructionModePartDetached(Vessel v, Part p)
		{
			if(part == p)
			{
				if(otherPort)
				{
					otherPort.otherPort = null;
					otherPort.dockedPartUId = 0;
					otherPort.fsm.RunEvent(otherPort.on_construction);
				}

				otherPort = null;
				dockedPartUId = 0;
				fsm.RunEvent(on_construction);
			}
		}

		////////////////////////////////////////
		// Functions

		public void SetupFSM()
		{
			fsm = new KerbalFSM();

			st_active = new KFSMState("Active");
			st_active.OnEnter = delegate(KFSMState from)
			{
				otherPort = null;
				dockedPartUId = 0;

				Events["TogglePort"].guiName = "Deactivate Port";
				Events["TogglePort"].active = true;

				Events["ToggleMode"].guiName = "Mode: Active";
				Events["ToggleMode"].active = (supportedModes != 2); // show only if multiple modes are possible

// FEHLER, ich such was
if((supportedModes & 2) == 0)
{
	Logger.Log("error, port in wrong mode", Logger.Level.Error);
	fsm.RunEvent(on_setpassive);
}

				DockStatus = st_active.name;
			};
			st_active.OnFixedUpdate = delegate
			{
				Vector3 distance; float angle;

				for(int i = 0; i < FlightGlobals.VesselsLoaded.Count; i++)
				{
					Vessel vessel = FlightGlobals.VesselsLoaded[i];

					if(vessel.packed
						/*|| (vessel == part.vessel)*/) // no docking to ourself is possible
						continue;

					for(int j = 0; j < vessel.dockingPorts.Count; j++)
					{
						PartModule partModule = vessel.dockingPorts[j];

						if((partModule.part == null)
						/*|| (partModule.part == part)*/ // no docking to ourself is possible
						|| (partModule.part.State == PartStates.DEAD))
							continue;

						ModuleIRBM _otherPort = partModule.GetComponent<ModuleIRBM>();

						if(_otherPort == null)
							continue;

						if(!nodeTypesAcceptedS.Contains(_otherPort.nodeType)
						|| !_otherPort.nodeTypesAcceptedS.Contains(nodeType))
							continue;

						if((_otherPort.fsm.CurrentState == _otherPort.st_active) && _otherPort.autoActivePassive)
							_otherPort.fsm.RunEvent(_otherPort.on_setpassive);

						if(_otherPort.fsm.CurrentState != _otherPort.st_passive)
							continue;

						distance = _otherPort.nodeTransform.position - nodeTransform.position;

						if(distance.magnitude < detectionDistance)
						{
							angle = Vector3.Angle(nodeTransform.forward, -_otherPort.nodeTransform.forward);

							DockDistance = distance.magnitude.ToString();
							DockAngle = "-";

							if((angle <= approachingAngle) && (distance.magnitude <= approachingDistance))
							{
								otherPort = _otherPort;
								dockedPartUId = otherPort.part.flightID;

								fsm.RunEvent(on_approach);
								otherPort.fsm.RunEvent(otherPort.on_approach_passive);
								return;
							}
						}
					}
				}

				DockDistance = "-";
				DockAngle = "-";
			};
			st_active.OnLeave = delegate(KFSMState to)
			{
				if((to != st_passive) && (to != st_disabled))
					Events["TogglePort"].active = false;

				if(to != st_passive)
					Events["ToggleMode"].active = false;
			};
			fsm.AddState(st_active);

			st_passive = new KFSMState("Passive");
			st_passive.OnEnter = delegate(KFSMState from)
			{
				otherPort = null;
				dockedPartUId = 0;

				Events["TogglePort"].guiName = "Deactivate Port";
				Events["TogglePort"].active = true;

				Events["ToggleMode"].guiName = "Mode: Passive";
				Events["ToggleMode"].active = (supportedModes != 1); // show only if multiple modes are possible

// FEHLER, ich such was
if((supportedModes & 1) == 0)
{
	Logger.Log("error, port in wrong mode", Logger.Level.Error);
	fsm.RunEvent(on_setactive);
}

				DockStatus = st_passive.name;
			};
			st_passive.OnFixedUpdate = delegate
			{
			};
			st_passive.OnLeave = delegate(KFSMState to)
			{
				if(to != st_disabled)
					Events["TogglePort"].active = false;

				if((to != st_active) && (to != st_disabled))
					Events["ToggleMode"].active = false;
			};
			fsm.AddState(st_passive);

			st_approaching = new KFSMState("Approaching");
			st_approaching.OnEnter = delegate(KFSMState from)
			{
				inCaptureDistance = false;

				otherPort.otherPort = this;
				otherPort.dockedPartUId = part.flightID;

			//	otherPort.fsm.RunEvent(otherPort.on_approach_passive); -> is done manually

				DockStatus = st_approaching.name;
			};
			st_approaching.OnFixedUpdate = delegate
			{
				Vector3 distance = otherPort.nodeTransform.position - nodeTransform.position;

				DockDistance = distance.magnitude.ToString();

				Vector3 tvref = nodeTransform.TransformDirection(dockingOrientation);
				Vector3 tv = otherPort.nodeTransform.TransformDirection(otherPort.dockingOrientation);
				float ang = Vector3.SignedAngle(tvref, tv, -nodeTransform.forward);

				ang = 360f + ang - (180f / snapCount);
				ang %= (360f / snapCount);
				ang -= (180f / snapCount);

				DockAngle = ang.ToString();

				if(distance.magnitude < captureDistance)
				{
					if(Mathf.Abs(ang) <= captureAngle)
					{
						if(!inCaptureDistance)
							Events["Latch"].active = true;

						inCaptureDistance = true;

						return;
					}
				}

				if(inCaptureDistance)
					Events["Latch"].active = false;

				inCaptureDistance = false;
				
				if(distance.magnitude < 1.5f * approachingDistance)
				{
					float angle = Vector3.Angle(nodeTransform.forward, -otherPort.nodeTransform.forward);

					if(angle <= approachingAngle)
						return;
				}

				otherPort.fsm.RunEvent(otherPort.on_distance_passive);
				fsm.RunEvent(on_distance);
			};
			st_approaching.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_approaching);

			st_approaching_passive = new KFSMState("Approaching (passive)");
			st_approaching_passive.OnEnter = delegate(KFSMState from)
			{
				DockStatus = st_approaching_passive.name;
			};
			st_approaching_passive.OnFixedUpdate = delegate
			{
			};
			st_approaching_passive.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_approaching_passive);

			st_latching = new KFSMState("Latching");
			st_latching.OnEnter = delegate(KFSMState from)
			{
				Events["Latch"].active = false;
				Events["Release"].active = true;

				BuildLatchJoint(otherPort);
				CalculateLatchJointTarget();

				float latchingDuration = Math.Max(
						(nodeTransform.position - otherPort.nodeTransform.position).magnitude / latchingSpeedTranslation,
						(Quaternion.Angle(LatchJointTargetRotation, Quaternion.identity) / latchingSpeedRotation));

				if(float.IsNaN(latchingDuration) || float.IsInfinity(latchingDuration))
					latchingDuration = 10;

				latchProgress = 1;
				latchProgressStep = TimeWarp.fixedDeltaTime / latchingDuration;

				LatchJointInitialPosition = LatchJoint.targetPosition;

				DockStatus = st_latching.name;
			};
			st_latching.OnFixedUpdate = delegate
			{
				if(latchProgress > latchProgressStep)
				{
					latchProgress -= latchProgressStep;

					LatchJoint.targetRotation = Quaternion.Slerp(LatchJointTargetRotation, Quaternion.identity, latchProgress);
					LatchJoint.targetPosition = Vector3.Lerp(LatchJointTargetPosition, LatchJointInitialPosition, latchProgress);
				}
				else
				{
					LatchJoint.targetRotation = LatchJointTargetRotation;
					LatchJoint.targetPosition = LatchJointTargetPosition;

					fsm.RunEvent(on_prelatched);
				}
			};
			st_latching.OnLeave = delegate(KFSMState to)
			{
				if(to != st_prelatched)
					Events["Release"].active = false;
			};
			fsm.AddState(st_latching);

			st_prelatched = new KFSMState("Pre Latched");
			st_prelatched.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;

				latchRelaxCounter = 10;

				DockStatus = st_prelatched.name;
			};
			st_prelatched.OnFixedUpdate = delegate
			{
				if(--latchRelaxCounter < 0)
				{
					fsm.RunEvent(on_latch);
					otherPort.fsm.RunEvent(otherPort.on_latch_passive);
				}
			};
			st_prelatched.OnLeave = delegate(KFSMState to)
			{
				if(to != st_latched)
					Events["Release"].active = false;
			};
			fsm.AddState(st_prelatched);
		
			st_latched = new KFSMState("Latched");
			st_latched.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;

				Events["Dock"].active = true;
				Events["Undock"].active = false;

				JointDrive angularDrive = new JointDrive { maximumForce = PhysicsGlobals.JointForce, positionSpring = 60000f, positionDamper = 0f };
				LatchJoint.angularXDrive = LatchJoint.angularYZDrive = LatchJoint.slerpDrive = angularDrive;

				JointDrive linearDrive = new JointDrive { maximumForce = PhysicsGlobals.JointForce, positionSpring = PhysicsGlobals.JointForce, positionDamper = 0f };
				LatchJoint.xDrive = LatchJoint.yDrive = LatchJoint.zDrive = linearDrive;

				DockStatus = st_latched.name;
			};
			st_latched.OnFixedUpdate = delegate
			{
			};
			st_latched.OnLeave = delegate(KFSMState to)
			{
				Events["Release"].active = false;

				Events["Dock"].active = false;
			};
			fsm.AddState(st_latched);

			st_latched_passive = new KFSMState("Latched (passive)");
			st_latched_passive.OnEnter = delegate(KFSMState from)
			{
				Events["MakeActive"].guiActive = ((supportedModes & 2) != 0) && ((otherPort.supportedModes & 1) != 0);

				DockStatus = st_latched_passive.name;
			};
			st_latched_passive.OnFixedUpdate = delegate
			{
			};
			st_latched_passive.OnLeave = delegate(KFSMState to)
			{
				Events["MakeActive"].guiActive = false;
			};
			fsm.AddState(st_latched_passive);

			st_docked = new KFSMState("Docked");
			st_docked.OnEnter = delegate(KFSMState from)
			{
				Events["Undock"].active = ((supportedModes & 2) != 0) || (directUndock && otherPort.directUndock);

				DockStatus = st_docked.name;
			};
			st_docked.OnFixedUpdate = delegate
			{
			};
			st_docked.OnLeave = delegate(KFSMState to)
			{
				Events["Undock"].active = false;
			};
			fsm.AddState(st_docked);

			st_preattached = new KFSMState("Attached");
			st_preattached.OnEnter = delegate(KFSMState from)
			{
				Events["Undock"].active = ((supportedModes & 2) != 0) || (directUndock && otherPort.directUndock);

				DockStatus = st_preattached.name;
			};
			st_preattached.OnFixedUpdate = delegate
			{
			};
			st_preattached.OnLeave = delegate(KFSMState to)
			{
				Events["Undock"].active = false;
			};
			fsm.AddState(st_preattached);

			st_disabled = new KFSMState("Inactive");
			st_disabled.OnEnter = delegate(KFSMState from)
			{
				Events["TogglePort"].guiName = "Activate Port";
				Events["TogglePort"].active = true;

				DockStatus = st_disabled.name;
			};
			st_disabled.OnFixedUpdate = delegate
			{
			};
			st_disabled.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_disabled);


			on_setactive = new KFSMEvent("Make Active");
			on_setactive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_setactive.GoToStateOnEvent = st_active;
			fsm.AddEvent(on_setactive, st_passive, st_disabled);

			on_setpassive = new KFSMEvent("Make Passive");
			on_setpassive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_setpassive.GoToStateOnEvent = st_passive;
			fsm.AddEvent(on_setpassive, st_active, st_disabled);


			on_approach = new KFSMEvent("Approaching");
			on_approach.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_approach.GoToStateOnEvent = st_approaching;
			fsm.AddEvent(on_approach, st_active);

			on_approach_passive = new KFSMEvent("Approaching");
			on_approach_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_approach_passive.GoToStateOnEvent = st_approaching_passive;
			fsm.AddEvent(on_approach_passive, st_passive);

			on_distance = new KFSMEvent("Distancing");
			on_distance.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_distance.GoToStateOnEvent = st_active;
			fsm.AddEvent(on_distance, st_approaching, st_docked, st_preattached);

			on_distance_passive = new KFSMEvent("Distanced");
			on_distance_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_distance_passive.GoToStateOnEvent = st_passive;
			fsm.AddEvent(on_distance_passive, st_approaching_passive);

			on_latching = new KFSMEvent("Latch");
			on_latching.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latching.GoToStateOnEvent = st_latching;
			fsm.AddEvent(on_latching, st_approaching);

			on_prelatched = new KFSMEvent("Pre Latch");
			on_prelatched.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_prelatched.GoToStateOnEvent = st_prelatched;
			fsm.AddEvent(on_prelatched, st_latching);

			on_latch = new KFSMEvent("Latched");
			on_latch.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latch.GoToStateOnEvent = st_latched;
			fsm.AddEvent(on_latch, st_prelatched, st_latched_passive);

			on_latch_passive = new KFSMEvent("Latched");
			on_latch_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latch_passive.GoToStateOnEvent = st_latched_passive;
			fsm.AddEvent(on_latch_passive, st_approaching_passive, st_latched);


			on_release = new KFSMEvent("Release");
			on_release.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_release.GoToStateOnEvent = st_active;
			fsm.AddEvent(on_release, st_latched);

			on_release_passive = new KFSMEvent("Release (passive)");
			on_release_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_release_passive.GoToStateOnEvent = st_passive;
			fsm.AddEvent(on_release_passive, st_latched_passive);


			on_dock = new KFSMEvent("Perform docking");
			on_dock.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_dock.GoToStateOnEvent = st_docked;
			fsm.AddEvent(on_dock, st_latched);

			on_dock_passive = new KFSMEvent("Dock");
			on_dock_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_dock_passive.GoToStateOnEvent = st_docked;
			fsm.AddEvent(on_dock_passive, st_latched_passive);

			on_undock = new KFSMEvent("Undock");
			on_undock.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_undock.GoToStateOnEvent = st_latched;
			fsm.AddEvent(on_undock, st_docked, st_preattached);

			on_undock_passive = new KFSMEvent("Undock");
			on_undock_passive.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_undock_passive.GoToStateOnEvent = st_latched_passive;
			fsm.AddEvent(on_undock_passive, st_docked, st_preattached);


			on_disable = new KFSMEvent("Disable");
			on_disable.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_disable.GoToStateOnEvent = st_disabled;
			fsm.AddEvent(on_disable, st_active, st_passive);


			on_construction = new KFSMEvent("Construction");
			on_construction.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_construction.GoToStateOnEvent = st_disabled;
			fsm.AddEvent(on_construction, st_active, st_passive, st_approaching, st_approaching_passive, st_latching, st_prelatched, st_latched, st_latched_passive, st_docked, st_preattached);
		}

		private void BuildLatchJoint(ModuleIRBM port)
		{
		// FEHLER, müsste doch schon gesetzt sein... aber gut...
			otherPort = port;
			dockedPartUId = otherPort.part.flightID;

			otherPort.otherPort = this;
			otherPort.dockedPartUId = part.flightID;

			// Joint
			ConfigurableJoint joint = gameObject.AddComponent<ConfigurableJoint>();

			joint.connectedBody = otherPort.part.Rigidbody;

			joint.breakForce = joint.breakTorque = Mathf.Infinity;
// FEHLER FEHLER -> breakForce min von beiden und torque auch

			// we calculate with the "stack" force -> thus * 4f and not * 1.6f

			float breakingForceModifier = 1f;
			float breakingTorqueModifier = 1f;

			float defaultLinearForce = Mathf.Min(part.breakingForce, otherPort.part.breakingForce) *
				breakingForceModifier * 4f;

			float defaultTorqueForce = Mathf.Min(part.breakingTorque, otherPort.part.breakingTorque) *
				breakingTorqueModifier * 4f;

			joint.breakForce = defaultLinearForce;
			joint.breakTorque = defaultTorqueForce;


			joint.xMotion = joint.yMotion = joint.zMotion = ConfigurableJointMotion.Free;
			joint.angularXMotion = joint.angularYMotion = joint.angularZMotion = ConfigurableJointMotion.Free;

			JointDrive drive =
				new JointDrive
				{
					positionSpring = 100f,
					positionDamper = 0f,
					maximumForce = 100f
				};

			joint.angularXDrive = joint.angularYZDrive = joint.slerpDrive = drive;
			joint.xDrive = joint.yDrive = joint.zDrive = drive;

			LatchJoint = joint;

			DockDistance = "-";
			DockAngle = "-";
		}

		private void CalculateLatchJointTarget()
		{
			Vector3 targetPosition; Quaternion targetRotation;
			DockingHelper.CalculateDockingPositionAndRotation(this, otherPort, out targetPosition, out targetRotation);

			// convert values from org-values to real values (for latching we need real values, for docking org-values)
			targetPosition +=
				otherPort.GetPart().transform.position
				- (otherPort.GetPart().vessel.transform.position + otherPort.GetPart().vessel.transform.rotation * otherPort.GetPart().orgPos);

			targetRotation *=
				Quaternion.Inverse(otherPort.GetPart().vessel.transform.rotation * otherPort.GetPart().orgRot)
				* otherPort.GetPart().transform.rotation;

			// invert both values
			LatchJointTargetPosition = -transform.InverseTransformPoint(targetPosition);
			LatchJointTargetRotation = Quaternion.Inverse(Quaternion.Inverse(transform.rotation) * targetRotation);
		}

		private void DestroyLatchJoint()
		{
			// Joint
			Destroy(LatchJoint);
			LatchJoint = null;

			// FEHLER, nur mal so 'ne Idee... weiss nicht ob das gut ist

			vessel.ResetRBAnchor();
			if(otherPort) otherPort.vessel.ResetRBAnchor();
		}

		////////////////////////////////////////
		// Update-Functions

		public void FixedUpdate()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(vessel && !vessel.packed)
				{
					if((fsm != null) && fsm.Started)
						fsm.FixedUpdateFSM();
				}
			}
		}

		public void Update()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(vessel && !vessel.packed)
				{
					if((fsm != null) && fsm.Started)
						fsm.UpdateFSM();

					if(FlightGlobals.fetch.VesselTarget == (ITargetable)this)
					{
						evtSetAsTarget.active = false;
						evtUnsetTarget.active = true;

						if(FlightGlobals.ActiveVessel == vessel)
							FlightGlobals.fetch.SetVesselTarget(null);
						else if((FlightGlobals.ActiveVessel.transform.position - nodeTransform.position).sqrMagnitude > 40000f)
							FlightGlobals.fetch.SetVesselTarget(vessel);
					}
					else
					{
						evtSetAsTarget.active = true;
						evtUnsetTarget.active = false;
					}
				}
			}
		}

		public void LateUpdate()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(vessel && !vessel.packed)
				{
					if((fsm != null) && fsm.Started)
						fsm.LateUpdateFSM();
				}
			}
		}

		////////////////////////////////////////
		// Context Menu

		[KSPField(guiName = "BM status", isPersistant = false, guiActive = true, guiActiveUnfocused = true, unfocusedRange = 20)]
		public string DockStatus = "Inactive";

		[KSPField(guiName = "BM distance", isPersistant = false, guiActive = true)]
		public string DockDistance;

		[KSPField(guiName = "BM angle", isPersistant = false, guiActive = true)]
		public string DockAngle;

		public void Enable()
		{
			if((supportedModes & 1) != 0)
				fsm.RunEvent(on_setpassive);
			else if((supportedModes & 2) != 0)
				fsm.RunEvent(on_setactive);
		}

		public void Disable()
		{
			fsm.RunEvent(on_disable);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, guiName = "Deactivate Port")]
		public void TogglePort()
		{
			if(fsm.CurrentState == st_disabled)
				Enable();
			else
				Disable();
		}

		public void SetActive()
		{
			if((supportedModes & 2) != 0)
				fsm.RunEvent(on_setactive);
		}

		public void SetPassive()
		{
			if((supportedModes & 1) != 0)
				fsm.RunEvent(on_setpassive);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, guiName = "Mode")]
		public void ToggleMode()
		{
			if(fsm.CurrentState == st_passive)
				SetActive();
			else if(fsm.CurrentState == st_active)
				SetPassive();
		}

// FEHLER, nicht so tolle Funktion
		[KSPEvent(guiActive = false, guiActiveUnfocused = false, guiName = "Make Active")]
		public void MakeActive()
		{
			otherPort.DestroyLatchJoint();

			BuildLatchJoint(otherPort);
			CalculateLatchJointTarget();

			LatchJointInitialPosition = LatchJoint.targetPosition;
// FEHLER, evtl. nicht nötig, dieser Wert hier? wobei... evtl. schon, weil wir was laufen lassen und sonst im Arsch wären?

			fsm.RunEvent(on_latch);
			otherPort.fsm.RunEvent(otherPort.on_latch_passive);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Latch")]
		public void Latch()
		{
			fsm.RunEvent(on_latching);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Release")]
		public void Release()
		{
			DestroyLatchJoint();

			if(otherPort != null)
				otherPort.fsm.RunEvent(otherPort.on_release_passive);

			fsm.RunEvent(on_release);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Dock")]
		public void Dock()
		{
			Debug.Log("Docking to vessel " + otherPort.vessel.GetDisplayName(), gameObject);

			dockedPartUId = otherPort.part.flightID;

			otherPort.otherPort = this;
			otherPort.dockedPartUId = part.flightID;

			DockingHelper.SaveCameraPosition(part);
			DockingHelper.SuspendCameraSwitch(10);

			if(otherPort.vessel == Vessel.GetDominantVessel(vessel, otherPort.vessel))
				DockingHelper.DockVessels(this, otherPort);
			else
				DockingHelper.DockVessels(otherPort, this);

			DockingHelper.RestoreCameraPosition(part);

			Destroy(LatchJoint);
			LatchJoint = null;

			fsm.RunEvent(on_dock);
			otherPort.fsm.RunEvent(otherPort.on_dock_passive);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, externalToEVAOnly = true, unfocusedRange = 2f, guiName = "#autoLOC_6001445")]
		public void Undock()
		{
			Vessel oldvessel = vessel;
			uint referenceTransformId = vessel.referenceTransformId;

			DockingHelper.SaveCameraPosition(part);
			DockingHelper.SuspendCameraSwitch(10);

			DockingHelper.UndockVessels(this, otherPort);

			DockingHelper.RestoreCameraPosition(part);

			if(!directUndock || !otherPort.directUndock)
			{
				// when we are here, this port must be the active port

				BuildLatchJoint(otherPort);
				CalculateLatchJointTarget();
			}

			otherPort.fsm.RunEvent(((supportedModes & 2) != 0) ? otherPort.on_undock_passive : otherPort.on_undock);
			fsm.RunEvent(((supportedModes & 2) != 0) ? on_undock : on_undock_passive);

			if(directUndock && otherPort.directUndock)
			{
				otherPort.fsm.RunEvent(((supportedModes & 2) != 0) ? otherPort.on_release_passive : otherPort.on_release);
				fsm.RunEvent(((supportedModes & 2) != 0) ? on_release : on_release_passive);
			}

			if(oldvessel == FlightGlobals.ActiveVessel)
			{
				if(vessel[referenceTransformId] == null)
					StartCoroutine(WaitAndSwitchFocus());
			}
		}

		public IEnumerator WaitAndSwitchFocus()
		{
			yield return null;

			DockingHelper.SaveCameraPosition(part);

			FlightGlobals.ForceSetActiveVessel(vessel);
			FlightInputHandler.SetNeutralControls();

			DockingHelper.RestoreCameraPosition(part);
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "#autoLOC_236028")]
		public void EnableXFeed()
		{
			Events["EnableXFeed"].active = false;
			Events["DisableXFeed"].active = true;
			bool fuelCrossFeed = part.fuelCrossFeed;
			part.fuelCrossFeed = (crossfeed = true);
			if(fuelCrossFeed != crossfeed)
				GameEvents.onPartCrossfeedStateChange.Fire(part);
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "#autoLOC_236030")]
		public void DisableXFeed()
		{
			Events["EnableXFeed"].active = true;
			Events["DisableXFeed"].active = false;
			bool fuelCrossFeed = part.fuelCrossFeed;
			part.fuelCrossFeed = (crossfeed = false);
			if(fuelCrossFeed != crossfeed)
				GameEvents.onPartCrossfeedStateChange.Fire(part);
		}

		////////////////////////////////////////
		// Actions

		[KSPAction("Enable")]
		public void EnableAction(KSPActionParam param)
		{ Enable(); }

		[KSPAction("Disable")]
		public void DisableAction(KSPActionParam param)
		{ Disable(); }

		[KSPAction("SetActive")]
		public void SetActiveAction(KSPActionParam param)
		{ SetActive(); }

		[KSPAction("SetPassive")]
		public void SetPassiveAction(KSPActionParam param)
		{ SetPassive(); }

		[KSPAction("Dock", activeEditor = false)]
		public void DockAction(KSPActionParam param)
		{ Dock(); }

		[KSPAction("#autoLOC_6001444", activeEditor = false)]
		public void UndockAction(KSPActionParam param)
		{ Undock(); }

		[KSPAction("#autoLOC_236028")]
		public void EnableXFeedAction(KSPActionParam param)
		{ EnableXFeed(); }

		[KSPAction("#autoLOC_236030")]
		public void DisableXFeedAction(KSPActionParam param)
		{ DisableXFeed(); }

		[KSPAction("#autoLOC_236032")]
		public void ToggleXFeedAction(KSPActionParam param)
		{
			if(crossfeed)
				DisableXFeed();
			else
				EnableXFeed();
		}

		////////////////////////////////////////
		// Reference / Target

		[KSPEvent(guiActive = false, guiActiveUnfocused = true, externalToEVAOnly = false, unfocusedRange = 200f, guiName = "#autoLOC_6001448")]
		public void SetAsTarget()
		{
			FlightGlobals.fetch.SetVesselTarget(this);
		}

		[KSPEvent(guiActive = false, guiActiveUnfocused = true, externalToEVAOnly = false, unfocusedRange = 200f, guiName = "#autoLOC_6001449")]
		public void UnsetTarget()
		{
			FlightGlobals.fetch.SetVesselTarget(null);
		}

		////////////////////////////////////////
		// IDockable

		private DockInfo dockInfo;

		public Part GetPart()
		{ return part; }

		public Transform GetNodeTransform()
		{ return nodeTransform; }

		public Vector3 GetDockingOrientation()
		{ return dockingOrientation; }

		public int GetSnapCount()
		{ return snapCount; }

		public DockInfo GetDockInfo()
		{ return dockInfo; }

		public void SetDockInfo(DockInfo _dockInfo)
		{
			dockInfo = _dockInfo;

			if(dockInfo == null)
			{
				dockedType = 0;
				vesselInfo = null;
			}
			else if(dockInfo.part == (IDockable)this)
			{
				dockedType = 0;
				vesselInfo = dockInfo.vesselInfo;
			}
			else
			{
				dockedType = 1;
				vesselInfo = dockInfo.targetVesselInfo;
			}
		}

		// returns true, if the port is (passive and) ready to dock with an other (active) port
		public bool IsReadyFor(IDockable otherPort)
		{
			if(otherPort != null)
			{
				ModuleIRBM _otherPort = otherPort.GetPart().GetComponent<ModuleIRBM>();

				if(!_otherPort)
					return false;

				if(!nodeTypesAcceptedS.Contains(_otherPort.nodeType)
				|| !_otherPort.nodeTypesAcceptedS.Contains(nodeType))
					return false;
			}

			return (fsm.CurrentState == st_passive) || ((fsm.CurrentState == st_active) && autoActivePassive);
		}

		public ITargetable GetTargetable()
		{
			return (ITargetable)this;
		}

		public bool IsDocked()
		{
			return ((fsm.CurrentState == st_docked) || (fsm.CurrentState == st_preattached));
		}

		public IDockable GetOtherDockable()
		{
			return IsDocked() ? (IDockable)otherPort : null;
		}

		////////////////////////////////////////
		// ITargetable

		public Transform GetTransform()
		{
			return nodeTransform;
		}

		public Vector3 GetObtVelocity()
		{
			return base.vessel.obt_velocity;
		}

		public Vector3 GetSrfVelocity()
		{
			return base.vessel.srf_velocity;
		}

		public Vector3 GetFwdVector()
		{
			return nodeTransform.forward;
		}

		public Vessel GetVessel()
		{
			return vessel;
		}

		public string GetName()
		{
			return portName;
		}

		public string GetDisplayName()
		{
			return GetName();
		}

		public Orbit GetOrbit()
		{
			return vessel.orbit;
		}

		public OrbitDriver GetOrbitDriver()
		{
			return vessel.orbitDriver;
		}

		public VesselTargetModes GetTargetingMode()
		{
			return VesselTargetModes.DirectionVelocityAndOrientation;
		}

		public bool GetActiveTargetable()
		{
			return false;
		}

		private DockingPortRenameDialog renameDialog;

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, guiName = "Rename Port")]
		public void Rename()
		{
			InputLockManager.SetControlLock("dockingPortRenameDialog");

			renameDialog = DockingPortRenameDialog.Spawn(portName, onPortRenameAccept, onPortRenameCancel);
		}

		private void onPortRenameAccept(string newPortName)
		{
			portName = newPortName;
			onPortRenameCancel();
		}

		private void onPortRenameCancel()
		{
			InputLockManager.RemoveControlLock("dockingPortRenameDialog");
		}

		////////////////////////////////////////
		// IModuleInfo

		string IModuleInfo.GetModuleTitle()
		{
			return "Berthing Mechanism";
		}

		string IModuleInfo.GetInfo()
		{
/*
			StringBuilder sb = new StringBuilder();
			sb.AppendFormat("Attach strength (catched): {0:F0}\n", catchedBreakForce);
			sb.AppendFormat("Attach strength (latched): {0:F0}\n", latchedBreakForce);

			if(electricChargeRequiredLatching != 0f)
			{
				sb.Append("\n\n");
				sb.Append("<b><color=orange>Requires:</color></b>\n");
				
				if(electricChargeRequiredLatching != 0f)
					sb.AppendFormat("- <b>Electric Charge:</b> {0:F0}\n  (for latching)", electricChargeRequiredLatching);
			}

			return sb.ToString();*/
return ""; // FEHLER, fehlt
		}

		Callback<Rect> IModuleInfo.GetDrawModulePanelCallback()
		{
			return null;
		}

		string IModuleInfo.GetPrimaryField()
		{
			return null;
		}

		////////////////////////////////////////
		// Debug

#if DEBUG
	/*	
		private MultiLineDrawer ld;

		private String[] astrDebug;
		private int istrDebugPos;

		private void DebugInit()
		{
			ld = new MultiLineDrawer();
			ld.Create(null);

			astrDebug = new String[10240];
			istrDebugPos = 0;
		}

		private void DebugString(String s)
		{
			astrDebug[istrDebugPos] = s;
			istrDebugPos = (istrDebugPos + 1) % 10240;
		}

		private void DrawPointer(int idx, Vector3 p_vector)
		{
			ld.Draw(idx, Vector3.zero, p_vector);
		}

		private void DrawRelative(int idx, Vector3 p_from, Vector3 p_vector)
		{
			ld.Draw(idx, p_from, p_from + p_vector);
		}

		private void DrawAxis(int idx, Transform p_transform, Vector3 p_vector, bool p_relative, Vector3 p_off)
		{
			ld.Draw(idx, p_transform.position + p_off, p_transform.position + p_off
				+ (p_relative ? p_transform.TransformDirection(p_vector) : p_vector));
		}

		private void DrawAxis(int idx, Transform p_transform, Vector3 p_vector, bool p_relative)
		{ DrawAxis(idx, p_transform, p_vector, p_relative, Vector3.zero); }
	*/
#endif

	}
}
