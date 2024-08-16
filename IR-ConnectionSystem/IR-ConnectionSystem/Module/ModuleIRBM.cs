using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using KSP.IO;
using UnityEngine;

using IR_ConnectionSystem.Utility;
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
		public string controlTransformName = "";

		[KSPField(isPersistant = false), SerializeField]
		public Vector3 dockingOrientation = Vector3.zero; // defines the direction of the docking port (when docked at a 0° angle, these local vectors of two ports point into the same direction)

		[KSPField(isPersistant = false), SerializeField]
		public int snapCount = 1;


		[KSPField(isPersistant = false), SerializeField]
		public float detectionDistance = 5f;

		[KSPField(isPersistant = false), SerializeField]
		public float approachingDistance = 0.3f;

		[KSPField(isPersistant = false), SerializeField]
		public float captureDistance = 0.03f;


		[KSPField(isPersistant = false)]
		public bool gendered = false;

		[KSPField(isPersistant = false)]
		public bool genderFemale = false;

		[KSPField(isPersistant = false)]
		public int supportedModes = 3; // passive = 1, active = 2, both = 3

		[KSPField(isPersistant = false)]
		public string nodeType = "BM";

		[KSPField(isPersistant = false)]
		public float breakingForce = 100f;

		[KSPField(isPersistant = false)]
		public float breakingTorque = 100f;

		[KSPField(isPersistant = false)]
		public string nodeName = "";				// FEHLER, mal sehen wozu wir den dann nutzen könnten


		[KSPField(isPersistant = true)]
		public bool crossfeed = true;

		// Docking and Status

		public BaseEvent evtSetAsTarget;
		public BaseEvent evtUnsetTarget;

		public Transform nodeTransform;
		public Transform controlTransform;

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

/* FEHLER, Sound fehlt noch total -> ah und einige Servos spielen keinen Sound, was ist da falsch? -> hat nix mit LEE zu tun zwar

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

		public DockedVesselInfo vesselInfo;
		public bool docked = false; // true, if the vessel of the otherPort is and should be the same as our vessel

		private bool inCaptureDistance = false;

		private ConfigurableJoint CaptureJoint;

		private Quaternion CaptureJointTargetRotation;
		private Vector3 CaptureJointTargetPosition;

		private Vector3 CaptureJointWoherIchKomme;	// FEHLER, alles Müll hier

		private float _rotStep;
		float _transstep = 0.0005f;
		int iPos = 0;

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
			DebugInit();

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

			if(node.HasValue("docked"))
				docked = bool.Parse(node.GetValue("docked"));

			if(node.HasNode("DOCKEDVESSEL"))
			{
				vesselInfo = new DockedVesselInfo();
				vesselInfo.Load(node.GetNode("DOCKEDVESSEL"));
			}

// FEHLER, hier fehlt noch Zeugs

			if(node.HasValue("followOtherPort"))
			{
				followOtherPort = bool.Parse(node.GetValue("followOtherPort"));

				node.TryGetValue("otherPortRelativePosition", ref otherPortRelativePosition);
				node.TryGetValue("otherPortRelativeRotation", ref otherPortRelativeRotation);
			}

			part.fuelCrossFeed = crossfeed;
		}

		public override void OnSave(ConfigNode node)
		{
			base.OnSave(node);

			node.AddValue("state", (string)(((fsm != null) && (fsm.Started)) ? fsm.currentStateName : DockStatus));

			node.AddValue("dockUId", dockedPartUId);

			node.AddValue("docked", docked);

			if(vesselInfo != null)
				vesselInfo.Save(node.AddNode("DOCKEDVESSEL"));

// FEHLER, hier fehlt noch Zeugs

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

			evtSetAsTarget = base.Events["SetAsTarget"];
			evtUnsetTarget = base.Events["UnsetTarget"];

			if(state == StartState.Editor)
				return;

			GameEvents.onVesselGoOnRails.Add(OnPack);
			GameEvents.onVesselGoOffRails.Add(OnUnpack);

		//	GameEvents.onFloatingOriginShift.Add(OnFloatingOriginShift);

			GameEvents.OnEVAConstructionModePartDetached.Add(OnEVAConstructionModePartDetached);

			nodeTransform = base.part.FindModelTransform(nodeTransformName);
			if(!nodeTransform)
			{
				Debug.LogWarning("[Docking Node Module]: WARNING - No node transform found with name " + nodeTransformName, base.part.gameObject);
				return;
			}
			if(controlTransformName == string.Empty)
				controlTransform = base.part.transform;
			else
			{
				controlTransform = base.part.FindModelTransform(controlTransformName);
				if(!controlTransform)
				{
					Debug.LogWarning("[Docking Node Module]: WARNING - No control transform found with name " + controlTransformName, base.part.gameObject);
					controlTransform = base.part.transform;
				}
			}

			StartCoroutine(WaitAndInitialize(state));

			StartCoroutine(WaitAndInitializeDockingNodeFix());
		}

		// FEHLER, ist 'n Quickfix, solange der blöde Port noch drüber hängt im Part...
		public IEnumerator WaitAndInitializeDockingNodeFix()
		{
			ModuleDockingNode DockingNode = part.FindModuleImplementing<ModuleDockingNode>();

			if(DockingNode)
			{
				while((DockingNode.fsm == null) || (!DockingNode.fsm.Started))
					yield return null;

				DockingNode.fsm.RunEvent(DockingNode.on_disable);
			}
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

			Events["EnableXFeed"].active = !crossfeed;
			Events["DisableXFeed"].active = crossfeed;

			if(dockedPartUId != 0)
			{
				Part otherPart;

				while(!(otherPart = FlightGlobals.FindPartByID(dockedPartUId)))
					yield return null;

				otherPort = otherPart.GetComponent<ModuleIRBM>();

		// FEHLER, logo, das könnte auch er laden... aber... na ja...
				otherPort.otherPort = this;
				otherPort.dockedPartUId = part.flightID;
			}

/* FEHLER, Zeug reaktivieren und neu schreiben
			if((DockStatus == "Extending ring")
			|| (DockStatus == "Retracting ring")
			|| (DockStatus == "Searching")
			|| (DockStatus == "Approaching")
			|| (DockStatus == "Push ring")
			|| (DockStatus == "Restore ring"))
			{
				BuildRingObject();
				ActiveJoint = BuildActiveJoint();

				RingObject.transform.position = part.transform.TransformPoint(_state.ringPosition);
				RingObject.transform.rotation = part.transform.rotation * _state.ringRotation;

				extendPosition = _state.extendPosition;

				ActiveJoint.targetPosition = _state.activeJointTargetPosition;
				ActiveJoint.targetRotation = _state.activeJointTargetRotation;

				_pushStep = _state._pushStep;

				// Pack

				RingObject.GetComponent<Rigidbody>().isKinematic = true;
				RingObject.GetComponent<Rigidbody>().detectCollisions = false;

				RingObject.transform.parent = transform;
			}

			if(DockStatus == "Capturing")
			{
				BuildRingObject();
				ActiveJoint = BuildActiveJoint();

				RingObject.transform.position = otherPort.transform.TransformPoint(_state.originalRingObjectLocalPosition);
				RingObject.transform.rotation = otherPort.transform.rotation * _state.originalRingObjectLocalRotation;

				extendPosition = _state.extendPosition;

				ActiveJoint.targetPosition = _state.activeJointTargetPosition;
				ActiveJoint.targetRotation = _state.activeJointTargetRotation;

				_pushStep = _state._pushStep;

		// FEHLER, hier machen wir wieder einen super schwachen Joint und fangen neu an mit dem Latching... das ist so gewollt (im Moment zumindest)
				BuildCaptureJoint(otherPort);
				BuildCaptureJoint2();

				// Pack

				ringRelativePosition = RingObject.transform.localPosition;
				ringRelativeRotation = RingObject.transform.localRotation;

				RingObject.transform.parent = transform;

				otherPortRelativePosition = _state.otherPortRelativePosition;
				otherPortRelativeRotation = _state.otherPortRelativeRotation;

				followOtherPort = true;
			}

			if((DockStatus == "Captured")
			|| (DockStatus == "Retracting ring"))
			{
				BuildRingObject();
				ActiveJoint = BuildActiveJoint();

				RingObject.transform.position = otherPort.transform.TransformPoint(_state.originalRingObjectLocalPosition);
				RingObject.transform.rotation = otherPort.transform.rotation * _state.originalRingObjectLocalRotation;

				extendPosition = _state.extendPosition;

				ActiveJoint.targetPosition = _state.activeJointTargetPosition;
				ActiveJoint.targetRotation = _state.activeJointTargetRotation;

				_pushStep = _state._pushStep;

		// FEHLER, hier machen wir wieder einen super schwachen Joint und fangen neu an mit dem Latching... das ist so gewollt (im Moment zumindest)

				BuildCaptureJoint(otherPort);
				BuildCaptureJoint2();

				RingObject.transform.localPosition =
						_capturePositionB;

				RingObject.transform.localRotation =
						_captureRotationB;

				iCapturePosition = 25;

				float f, d;

				f = 10000f * iCapturePosition;
				d = 0.001f;

				JointDrive drive = new JointDrive
				{
					positionSpring = f,
					positionDamper = d,
					maximumForce = f
				};

				CaptureJoint.xDrive = drive;
				CaptureJoint.yDrive = drive;
				CaptureJoint.zDrive = drive;

				CaptureJoint.slerpDrive = drive;

				// Pack

				ringRelativePosition = RingObject.transform.localPosition;
				ringRelativeRotation = RingObject.transform.localRotation;

				RingObject.transform.parent = transform;

				otherPortRelativePosition = _state.otherPortRelativePosition;
				otherPortRelativeRotation = _state.otherPortRelativeRotation;

				followOtherPort = true;
			}

// FEHLER, fehlt noch total
			if(DockStatus == "Pre Latched")
			{
			}

*/
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

// FEHLER wenn ich nicht disabled bin, dann meinen GF disablen... so oder so... -> und das dort auch noch reinnehmen in die st_^'s

			SetupFSM();

			if((DockStatus == "Approaching")
			|| (DockStatus == "Latching")		// not required
			|| (DockStatus == "Pre Latched")	// not required
			|| (DockStatus == "Latched"))
			{
// FEHLER
//nope, ich muss alles bauen von dem Teil da... gut, auf den fsm von ihm kann ich zwar warten... das stimmt wohl
//ok, ansehen, dass wir's koordinieren

				if(otherPort != null)
				{
					while(!otherPort.part.started || (otherPort.fsm == null) || (!otherPort.fsm.Started))
						yield return null;
				}
			}

				// FEHLER, verfeinern, die Zustände
			if(DockStatus == "Latched")
			{
				BuildCaptureJoint(otherPort);
				BuildCaptureJoint2();
			}

			if(DockStatus == "Docked")
			{
				if(vessel == otherPort.vessel)
					docked = true;

				otherPort.DockStatus = "Docked";

				if(Vessel.GetDominantVessel(vessel, otherPort.vessel) == vessel)
					DockingHelper.OnLoad(this, vesselInfo, otherPort, otherPort.vesselInfo);
			}

			fsm.StartFSM(DockStatus);
		}

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
					if(followOtherPort)
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
				Events["ToggleMode"].active = true;
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

// FEHLER, gender beachten

						if(_otherPort.fsm.CurrentState != _otherPort.st_passive)
							continue;

						distance = _otherPort.nodeTransform.position - nodeTransform.position;

						if(distance.magnitude < detectionDistance)
						{
							angle = Vector3.Angle(nodeTransform.forward, -_otherPort.nodeTransform.forward);

							DockDistance = distance.magnitude.ToString();
							DockAngle = "-";

							if((angle <= 15f) && (distance.magnitude <= approachingDistance))
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
				Events["TogglePort"].active = false;

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
				Events["ToggleMode"].active = true;
			};
			st_passive.OnFixedUpdate = delegate
			{
			};
			st_passive.OnLeave = delegate(KFSMState to)
			{
				Events["TogglePort"].active = false;

				Events["ToggleMode"].active = false;
			};
			fsm.AddState(st_passive);

			st_approaching = new KFSMState("Approaching");
			st_approaching.OnEnter = delegate(KFSMState from)
			{
				Events["TogglePort"].active = false;

				inCaptureDistance = false;

				otherPort.otherPort = this;
				otherPort.dockedPartUId = part.flightID;

//				otherPort.fsm.RunEvent(otherPort.on_approach_passive);
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
					bool angleok = ((ang > -5f) && (ang < 5f));

					if(angleok)
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

					if(angle <= 15f)
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

				BuildCaptureJoint(otherPort);
				BuildCaptureJoint2();

				_transstep = 0.0005f / (nodeTransform.position - otherPort.nodeTransform.position).magnitude;

				CaptureJointWoherIchKomme = CaptureJoint.targetPosition;
			};
			st_latching.OnFixedUpdate = delegate
			{
				if(_rotStep > _transstep)
				{
					_rotStep -= _transstep;

					CaptureJoint.targetRotation = Quaternion.Slerp(CaptureJointTargetRotation, Quaternion.identity, _rotStep);

					Vector3 diff = otherPort.nodeTransform.position - nodeTransform.position;
					diff = CaptureJoint.transform.InverseTransformDirection(diff);

					if(diff.magnitude < 0.0005f)
						CaptureJoint.targetPosition -= diff;
					else
						CaptureJoint.targetPosition -= diff.normalized * 0.0005f;
	// FEHLER, etwas unschön, weil ich kein Slerp machen kann, weil ich mich vorher ausgerichtet habe... hmm... -> evtl. Basis rechnen, dann differenz davon und dann... dazwischen Slerpen?

// FEHLER, hab's doch noch neu gemacht... mal sehen ob's so stimmt oder zumindest etwas besser passt
CaptureJoint.targetPosition = Vector3.Slerp(CaptureJointTargetPosition, CaptureJointWoherIchKomme, _rotStep);
				}
				else
				{
					CaptureJoint.targetRotation = CaptureJointTargetRotation;
					CaptureJoint.targetPosition = CaptureJointTargetPosition;

					fsm.RunEvent(on_prelatched);
				}
			};
			st_latching.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_latching);

			st_prelatched = new KFSMState("Pre Latched");
			st_prelatched.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;

				iPos = 10;
			};
			st_prelatched.OnFixedUpdate = delegate
			{
				if(--iPos < 0)
				{
					fsm.RunEvent(on_latch);
					otherPort.fsm.RunEvent(otherPort.on_latch_passive);
				}
			};
			st_prelatched.OnLeave = delegate(KFSMState to)
			{
// FEHLER, evtl. noch relaxing machen, wenn gleiches Schiff?

/*
				DockToVessel(otherPort);

				Destroy(CaptureJoint);
				CaptureJoint = null;
*/
		//		otherPort.fsm.RunEvent(otherPort.on_dock);
			};
			fsm.AddState(st_prelatched);
		
			st_latched = new KFSMState("Latched");
			st_latched.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;

				Events["Dock"].active = true;
				Events["Undock"].active = false;

//if(from == st_prelatched) // FEHLER, quickfix... das schöner machen mal -> wenn ich vom Docked komme, dann renne ich auch hier rein
{
				JointDrive angularDrive = new JointDrive { maximumForce = PhysicsGlobals.JointForce, positionSpring = 60000f, positionDamper = 0f };
				CaptureJoint.angularXDrive = CaptureJoint.angularYZDrive = CaptureJoint.slerpDrive = angularDrive;

				JointDrive linearDrive = new JointDrive { maximumForce = PhysicsGlobals.JointForce, positionSpring = PhysicsGlobals.JointForce, positionDamper = 0f };
				CaptureJoint.xDrive = CaptureJoint.yDrive = CaptureJoint.zDrive = linearDrive;

	//			otherPort.fsm.RunEvent(otherPort.on_latch_passive);
}
			};
			st_latched.OnFixedUpdate = delegate
			{
			};
			st_latched.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_latched);

			st_latched_passive = new KFSMState("Latched (passive)");
			st_latched_passive.OnEnter = delegate(KFSMState from)
			{
				Events["MakeActive"].guiActive = ((supportedModes & 2) != 0);
			};
			st_latched_passive.OnFixedUpdate = delegate
			{
			};
			st_latched_passive.OnLeave = delegate(KFSMState to)
			{
				if(to == st_passive)
				{
					otherPort = null;
					dockedPartUId = 0;
				}

				Events["MakeActive"].guiActive = false;
			};
			fsm.AddState(st_latched_passive);

			st_docked = new KFSMState("Docked");
			st_docked.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = false;

				Events["Dock"].active = false;
				Events["Undock"].active = true;
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
				Events["Release"].active = false;

				Events["Undock"].active = true;
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

		// calculate position and orientation for st_capture
		void CalculateCaptureJointRotationAndPosition(ModuleIRBM port, out Quaternion rotation, out Vector3 position)
		{
			Vector3 tvref =
				transform.InverseTransformDirection(nodeTransform.TransformDirection(dockingOrientation));

			Vector3 portDockingOrientation = port.nodeTransform.TransformDirection(port.dockingOrientation);
			Vector3 tv = transform.InverseTransformDirection(portDockingOrientation);

			float angle = 0f;

			for(int i = 1; i < snapCount; i++)
			{
				float ff = (360f / snapCount) * i;

				Vector3 tv2 = transform.InverseTransformDirection(Quaternion.AngleAxis(ff, port.nodeTransform.forward) * portDockingOrientation);

				if(Vector3.Angle(tv, tvref) > Vector3.Angle(tv2, tvref))
				{
					tv = tv2;
					angle = ff;
				}
			}

			Quaternion qt = Quaternion.LookRotation(transform.InverseTransformDirection(nodeTransform.forward), transform.InverseTransformDirection(nodeTransform.TransformDirection(dockingOrientation)));
			Quaternion qc = Quaternion.LookRotation(transform.InverseTransformDirection(-port.nodeTransform.forward), tv);

			rotation = qt * Quaternion.Inverse(qc);


			Vector3 diff = port.nodeTransform.position - nodeTransform.position;
		//	Vector3 difflp = Vector3.ProjectOnPlane(diff, transform.forward);

			position = -transform.InverseTransformDirection(diff);
		}

		private void BuildCaptureJoint(ModuleIRBM port)
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

			CaptureJoint = joint;

			DockDistance = "-";
			DockAngle = "-";
		}

		private void BuildCaptureJoint2()
		{
			CalculateCaptureJointRotationAndPosition(otherPort, out CaptureJointTargetRotation, out CaptureJointTargetPosition);
			_rotStep = 1f;
		}

		private void DestroyCaptureJoint()
		{
			// Joint
			Destroy(CaptureJoint);
			CaptureJoint = null;

			// FEHLER, nur mal so 'ne Idee... weiss nicht ob das gut sit

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
/*
				if(vessel.packed && followOtherPort)
				{
					vessel.SetRotation(otherPort.part.transform.rotation * otherPortRelativeRotation, true);
					vessel.SetPosition(otherPort.part.transform.position + otherPort.part.transform.rotation * otherPortRelativePosition, false);
				//	vessel.IgnoreGForces(5);
				}
*/
			}
		}

		public void Update()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(vessel && !vessel.packed)
				{

					if((fsm != null) && fsm.Started)
					{
						fsm.UpdateFSM();
						DockStatus = fsm.currentStateName;
					}

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
			otherPort.DestroyCaptureJoint();

			BuildCaptureJoint(otherPort);
			BuildCaptureJoint2();

			CaptureJointWoherIchKomme = CaptureJoint.targetPosition;

			fsm.RunEvent(on_latch);
			otherPort.fsm.RunEvent(otherPort.on_latch_passive);
		}

	// das ist das pull-back und eine Drehung (gleichzeitig)
		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Latch")]
		public void Latch()
		{
			fsm.RunEvent(on_latching);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Release")]
		public void Release()
		{
			DestroyCaptureJoint();

			Events["Release"].active = false;
			Events["Latch"].active = false;
			Events["Dock"].active = false;

			if(otherPort != null)
				otherPort.fsm.RunEvent(otherPort.on_release_passive);

			fsm.RunEvent(on_release);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Dock")]
		public void Dock()
		{
			DockToVessel(otherPort);

			Destroy(CaptureJoint);
			CaptureJoint = null;

			fsm.RunEvent(on_dock);
			otherPort.fsm.RunEvent(otherPort.on_dock_passive);
		}

		public void DockToVessel(ModuleIRBM port)
		{
			Debug.Log("Docking to vessel " + port.vessel.GetDisplayName(), gameObject);

			otherPort = port;
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
		}

		private void DoUndock()
		{
			DockingHelper.SaveCameraPosition(part);
			DockingHelper.SuspendCameraSwitch(10);

			DockingHelper.UndockVessels(this, otherPort);

			BuildCaptureJoint(otherPort);
			BuildCaptureJoint2();

			DockingHelper.RestoreCameraPosition(part);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, externalToEVAOnly = true, unfocusedRange = 2f, guiName = "#autoLOC_6001445")]
		public void Undock()
		{
			Vessel oldvessel = vessel;
			uint referenceTransformId = vessel.referenceTransformId;

			DoUndock();

			otherPort.fsm.RunEvent(otherPort.on_undock_passive);
			fsm.RunEvent(on_undock);

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
				GameEvents.onPartCrossfeedStateChange.Fire(base.part);
		}

		[KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "#autoLOC_236030")]
		public void DisableXFeed()
		{
			Events["EnableXFeed"].active = true;
			Events["DisableXFeed"].active = false;
			bool fuelCrossFeed = base.part.fuelCrossFeed;
			base.part.fuelCrossFeed = (crossfeed = false);
			if(fuelCrossFeed != crossfeed)
				GameEvents.onPartCrossfeedStateChange.Fire(base.part);
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

		[KSPEvent(guiActive = true, guiName = "#autoLOC_6001447")]
		public void MakeReferenceTransform()
		{
			part.SetReferenceTransform(controlTransform);
			vessel.SetReferenceTransform(part);
		}

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
			vesselInfo =
				(dockInfo == null) ? null :
				((dockInfo.part == (IDockable)this) ? dockInfo.vesselInfo : dockInfo.targetVesselInfo);
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
			return "Common Berthing Mechanism"; // FEHLER, einbauen
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
	}
}
