using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using KSP.IO;
using UnityEngine;

using IR_ConnectionSystem.Utility;
using AttachmentAndDockingTools;

namespace IR_ConnectionSystem.Module
{
	// FEHLER, Crossfeed noch einrichten... und halt umbauen auf FSM? ... ja, zum Spass... shit ey

	// FEHLER, wir arbeiten bei den Events nie mit "OnCheckCondition" sondern lösen alle manuell aus... kann man sich fragen, ob das gut ist, aber so lange der Event nur von einem Zustand her kommen kann, spielt das wie keine Rolle

	/*
	 * captured = first contact, centering is running (no orientation)
	 * latched  = centering has been done, weak connection (not latched in the way we would call it in reality)
	 * docked   = latched, docked or docked_to_same_vessel (it may not be docked in the sense of KSP)
	*/

	public class ModuleIRLEE : PartModule, IDockable, IModuleInfo
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
		public float captureDistance = 0.06f;


		[KSPField(isPersistant = false)]
		public bool gendered = true;

		[KSPField(isPersistant = false)]
		public bool genderFemale = false;

		[KSPField(isPersistant = false)]
		public string nodeType = "GF";

		[KSPField(isPersistant = false)]
		public float breakingForce = 10f;

		[KSPField(isPersistant = false)]
		public float breakingTorque = 10f;

		[KSPField(isPersistant = false)]
		public string nodeName = "";				// FEHLER, mal sehen wozu wir den dann nutzen könnten


		[KSPField(isPersistant = true)]
		public bool autoCapture = false;

		[KSPField(isPersistant = true)]
		public bool crossfeed = true;

		// Docking and Status

		public Transform nodeTransform;
		public Transform controlTransform;

		public KerbalFSM fsm;

		public KFSMState st_active;			// "active" / "searching"

		public KFSMState st_approaching;	// port found

		public KFSMState st_capturing;		// we establish a first connection
		public KFSMState st_captured;		// we have a first, weak but stable and centered connection and the system is ready for orienting, pullback and latching

		public KFSMState st_latching;		// orienting and retracting in progress
		public KFSMState st_prelatched;		// ready to dock
		public KFSMState st_latched;		// docked

		public KFSMState st_released;		// after a capture or latch, the parts have been detached again -> maybe for an abort of the docking
		
		public KFSMState st_docked;			// docked or docked_to_same_vessel
		public KFSMState st_preattached;

		public KFSMState st_disabled;


		public KFSMEvent on_approach;
		public KFSMEvent on_distance;

		public KFSMEvent on_capture;
		public KFSMEvent on_captured;

		public KFSMEvent on_latch;
		public KFSMEvent on_prelatched;
		public KFSMEvent on_latched;

		public KFSMEvent on_release;

		public KFSMEvent on_dock;
		public KFSMEvent on_undock;

		public KFSMEvent on_enable;
		public KFSMEvent on_disable;

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

		public ModuleIRGF otherPort;
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

		private bool followOtherPort = false;			// FEHLER, temp noch drin, später raus

		private Vector3 otherPortRelativePosition;
		private Quaternion otherPortRelativeRotation;

		////////////////////////////////////////
		// Constructor

		public ModuleIRLEE()
		{
		}

		////////////////////////////////////////
		// Callbacks

		public override void OnAwake()
		{
			DebugInit();

		//	part.dockingPorts.AddUnique(this);
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

		public override void OnStart(StartState st)
		{
			base.OnStart(st);

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

			StartCoroutine(WaitAndInitialize(st));

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

			Events["AutoCapture"].active = false;
			Events["AutoCapture"].guiName = autoCapture ? "Auto Capturing" : "Manual Capturing";

			Events["Capture"].active = false;
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

				otherPort = otherPart.GetComponent<ModuleIRGF>();

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

			if(DockStatus == "Docked")
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
						ModuleIRGF _otherPort = node.attachedPart.GetComponent<ModuleIRGF>();

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

			if(DockStatus == "Docked")
			{
				if(vessel == otherPort.vessel)
					docked = true;

				otherPort.DockStatus = "Docked";
			}

// FEHLER wenn ich nicht disabled bin, dann meinen GF disablen... so oder so... -> und das dort auch noch reinnehmen in die st_^'s

			SetupFSM();

			if((DockStatus == "Approaching")
			|| (DockStatus == "Capturing")
			|| (DockStatus == "Capture released"))
			{
// FEHLER
//nope, ich muss alles bauen von dem Teil da... gut, auf den fsm von ihm kann ich zwar warten... das stimmt wohl
//ok, ansehen, dass wir's koordinieren

				if(otherPort != null)
				{
					while((otherPort.fsm == null) || (!otherPort.fsm.Started))
						yield return null;
				}
			}

			fsm.StartFSM(DockStatus);
		}

		public void Start()
		{
			GameEvents.onVesselGoOnRails.Add(OnPack);
			GameEvents.onVesselGoOffRails.Add(OnUnpack);

		//	GameEvents.onFloatingOriginShift.Add(OnFloatingOriginShift);
		}

		public void OnDestroy()
		{
			GameEvents.onVesselGoOnRails.Remove(OnPack);
			GameEvents.onVesselGoOffRails.Remove(OnUnpack);

		//	GameEvents.onFloatingOriginShift.Remove(OnFloatingOriginShift);
		}

		private void OnPack(Vessel v)
		{
			if(vessel == v)
			{
				if((DockStatus == "Captured")
				|| (DockStatus == "Latched"))
				{
					followOtherPort = true;

					ShipCoordinator.Register(part, otherPort.part, true, out otherPortRelativePosition, out otherPortRelativeRotation);
				}
			}
		}

		private void OnUnpack(Vessel v)
		{
			if(vessel == v)
			{
				if((DockStatus == "Captured")
				|| (DockStatus == "Latched"))
				{
					followOtherPort = false;

					ShipCoordinator.Unregister(vessel);
				}

		//		StartCoroutine(OnUnpackDelayed());
			}
		}

		////////////////////////////////////////
		// Functions

		public void SetupFSM()
		{
			fsm = new KerbalFSM();

			st_active = new KFSMState("Ready");
			st_active.OnEnter = delegate(KFSMState from)
			{
				otherPort = null;
				dockedPartUId = 0;

				Events["TogglePort"].guiName = "Deactivate Port";
				Events["TogglePort"].active = true;

				Events["AutoCapture"].guiName = autoCapture ? "Auto Capturing" : "Manual Capturing";
				Events["AutoCapture"].active = true;
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

						ModuleIRGF _otherPort = partModule.GetComponent<ModuleIRGF>();

						if(_otherPort == null)
							continue;

						if(_otherPort.fsm.CurrentState != _otherPort.st_passive)
							continue;

						distance = _otherPort.nodeTransform.position - nodeTransform.position;

						if(distance.magnitude < detectionDistance)
						{
							DockDistance = distance.magnitude.ToString();

							angle = Vector3.Angle(nodeTransform.forward, -_otherPort.nodeTransform.forward);

							if((angle <= 15f) && (distance.magnitude <= approachingDistance))
							{
								otherPort = _otherPort;
								dockedPartUId = otherPort.part.flightID;

								fsm.RunEvent(on_approach);
								return;
							}
						}
					}
				}
			};
			st_active.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_active);

			st_approaching = new KFSMState("Approaching");
			st_approaching.OnEnter = delegate(KFSMState from)
			{
				Events["TogglePort"].active = false;

				Events["AutoCapture"].guiName = autoCapture ? "Auto Capturing" : "Manual Capturing";
				Events["AutoCapture"].active = true;

				inCaptureDistance = false;

				otherPort.otherPort = this;
				otherPort.dockedPartUId = part.flightID;

				otherPort.fsm.RunEvent(otherPort.on_approach_passive);
			};
			st_approaching.OnFixedUpdate = delegate
			{
				Vector3 distance = otherPort.nodeTransform.position - nodeTransform.position;

				if(distance.magnitude < captureDistance)
				{
					Vector3 tvref = nodeTransform.TransformDirection(dockingOrientation);
					Vector3 tv = otherPort.nodeTransform.TransformDirection(otherPort.dockingOrientation);
					float ang = Vector3.Angle(tvref, tv);

					bool angleok = false;

					for(int i = 0; i < snapCount; i++)
					{
						float ff = (360f / snapCount) * i;

						if((ang > ff - 5f) && (ang < ff + 5f))
							angleok = true;
					}

					if(angleok)
					{
						if(autoCapture)
						{
							fsm.RunEvent(on_capture);

							return;
						}

						if(!inCaptureDistance)
							Events["Capture"].active = true;

						inCaptureDistance = true;

						return;
					}
				}

				if(inCaptureDistance)
					Events["Capture"].active = false;

				inCaptureDistance = false;
				
				if(distance.magnitude < 1.5f * approachingDistance)
				{
					float angle = Vector3.Angle(nodeTransform.forward, -otherPort.nodeTransform.forward);

					if(angle <= 15f)
					{
						DockDistance = distance.magnitude.ToString();
						return;
					}
				}

				fsm.RunEvent(on_distance);
			};
			st_approaching.OnLeave = delegate(KFSMState to)
			{
				if(to == st_active)
				{
					inCaptureDistance = false;

					otherPort.fsm.RunEvent(otherPort.on_distance_passive);
				}
			};
			fsm.AddState(st_approaching);

			st_capturing = new KFSMState("Capturing");
			st_capturing.OnEnter = delegate(KFSMState from)
			{
				Events["TogglePort"].active = false;
				Events["AutoCapture"].active = false;

				Events["Capture"].active = false;

				BuildCaptureJoint(otherPort);
				BuildCaptureJoint2();

				Events["Release"].active = true;

				otherPort.fsm.RunEvent(otherPort.on_capture_passive);
			};
			st_capturing.OnFixedUpdate = delegate
			{
				// distance from axis
				Vector3 diff = otherPort.nodeTransform.position - nodeTransform.position;
				Vector3 diffp = Vector3.ProjectOnPlane(diff, nodeTransform.forward);
				Vector3 diffpl = Quaternion.Inverse(CaptureJoint.transform.rotation) * diffp;

				if(diffpl.magnitude >= 0.0005f)
					CaptureJoint.targetPosition -= diffpl.normalized * 0.0005f;
				else
				{
					CaptureJoint.targetPosition -= diffpl;

					fsm.RunEvent(on_captured);
				}
			};
			st_capturing.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_capturing);

			st_captured = new KFSMState("Captured");
			st_captured.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;
				Events["Latch"].active = true;
			};
			st_captured.OnFixedUpdate = delegate
			{
			};
			st_captured.OnLeave = delegate(KFSMState to)
			{
				Events["Latch"].active = false;
			};
			fsm.AddState(st_captured);
		
			st_latching = new KFSMState("Latching");
			st_latching.OnEnter = delegate(KFSMState from)
			{
				Events["Release"].active = true;

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
					fsm.RunEvent(on_latched);
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

				otherPort.fsm.RunEvent(otherPort.on_latch_passive);
}
			};
			st_latched.OnFixedUpdate = delegate
			{
			};
			st_latched.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_latched);


			st_released = new KFSMState("Released");
			st_released.OnEnter = delegate(KFSMState from)
			{
				DestroyCaptureJoint();

				Events["Release"].active = false;
				Events["Latch"].active = false;
				Events["Dock"].active = false;

				if(otherPort != null)
					otherPort.fsm.RunEvent(otherPort.on_release_passive);
			};
			st_released.OnFixedUpdate = delegate
			{
				float distance = (otherPort.nodeTransform.position - nodeTransform.position).magnitude;

				DockDistance = distance.ToString();

				if(distance > 2f * approachingDistance)
				{
					otherPort = null;
					dockedPartUId = 0;

					fsm.RunEvent(on_distance);
				}
			};
			st_released.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_released);
		
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
				Events["Release"].active = true;
				Events["Undock"].active = true;
			};
			st_preattached.OnFixedUpdate = delegate
			{
			};
			st_preattached.OnLeave = delegate(KFSMState to)
			{
				Events["Release"].active = false;
				Events["Undock"].active = false;
			};
			fsm.AddState(st_preattached);

			st_disabled = new KFSMState("Inactive");
			st_disabled.OnEnter = delegate(KFSMState from)
			{
				Events["TogglePort"].guiName = "Activate Port";
				Events["TogglePort"].active = true;

				Events["AutoCapture"].active = false;
			};
			st_disabled.OnFixedUpdate = delegate
			{
			};
			st_disabled.OnLeave = delegate(KFSMState to)
			{
			};
			fsm.AddState(st_disabled);


			on_approach = new KFSMEvent("Approaching");
			on_approach.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_approach.GoToStateOnEvent = st_approaching;
			fsm.AddEvent(on_approach, st_active);

			on_distance = new KFSMEvent("Distancing");
			on_distance.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_distance.GoToStateOnEvent = st_active;
			fsm.AddEvent(on_distance, st_approaching, st_released);

			on_capture = new KFSMEvent("Capture");
			on_capture.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_capture.GoToStateOnEvent = st_capturing;
			fsm.AddEvent(on_capture, st_approaching);

			on_captured = new KFSMEvent("Captured");
			on_captured.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_captured.GoToStateOnEvent = st_captured;
			fsm.AddEvent(on_captured, st_capturing);
			
			on_latch = new KFSMEvent("Latch");
			on_latch.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latch.GoToStateOnEvent = st_latching;
			fsm.AddEvent(on_latch, st_captured, st_capturing, st_approaching); // FEHLER, nicht sicher ob das auch von st_capturing und st_approaching her möglich sein muss... aber, vorerst ist es mal drin

			on_prelatched = new KFSMEvent("Pre Latch");
			on_prelatched.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_prelatched.GoToStateOnEvent = st_prelatched;
			fsm.AddEvent(on_prelatched, st_latching);

			on_latched = new KFSMEvent("Latched");
			on_latched.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_latched.GoToStateOnEvent = st_latched;
			fsm.AddEvent(on_latched, st_prelatched);

			on_release = new KFSMEvent("Released");
			on_release.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_release.GoToStateOnEvent = st_released;
			fsm.AddEvent(on_release, st_captured, st_latched);


			on_dock = new KFSMEvent("Perform docking");
			on_dock.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_dock.GoToStateOnEvent = st_docked;
			fsm.AddEvent(on_dock, st_latched);

			on_undock = new KFSMEvent("Undock");
			on_undock.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_undock.GoToStateOnEvent = st_latched;
			fsm.AddEvent(on_undock, st_docked, st_preattached);


			on_enable = new KFSMEvent("Enable");
			on_enable.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_enable.GoToStateOnEvent = st_active;
			fsm.AddEvent(on_enable, st_disabled);

			on_disable = new KFSMEvent("Disable");
			on_disable.updateMode = KFSMUpdateMode.MANUAL_TRIGGER;
			on_disable.GoToStateOnEvent = st_disabled;
			fsm.AddEvent(on_disable, st_active);
		}

		// calculate position and orientation for st_capture
		void CalculateCaptureJointRotationAndPosition(ModuleIRGF port, out Quaternion rotation, out Vector3 position)
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

		private void BuildCaptureJoint(ModuleIRGF port)
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
				if(!vessel.packed)
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
				if(!vessel.packed)
				{

				if((fsm != null) && fsm.Started)
				{
					fsm.UpdateFSM();
					DockStatus = fsm.currentStateName;
				}

				}
			}
		}

		public void LateUpdate()
		{
			if(HighLogic.LoadedSceneIsFlight)
			{
				if(!vessel.packed)
				{

				if((fsm != null) && fsm.Started)
					fsm.LateUpdateFSM();
				}
			}
		}

		////////////////////////////////////////
		// Context Menu

		[KSPField(guiName = "LEE status", isPersistant = false, guiActive = true, guiActiveUnfocused = true, unfocusedRange = 20)]
		public string DockStatus = "Inactive";

		[KSPField(guiName = "LEE distance", isPersistant = false, guiActive = true)]
		public string DockDistance;

// FEHLER, DockAngle noch? und evtl. die Anzeige als... weiss ned... evtl. immer zulassen? aber nur wenn approaching? also im DockingNodeEx?? weil der zählt schon ohne approaching zu sein

		public void Enable()
		{
			fsm.RunEvent(on_enable);
		}

		public void Disable()
		{
			fsm.RunEvent(on_disable);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, guiName = "Deactivate Port")]
		public void TogglePort()
		{
			if(fsm.CurrentState == st_disabled)
				fsm.RunEvent(on_enable);
			else
				fsm.RunEvent(on_disable);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, guiName = "Auto Capturing")]
		public void AutoCapture()
		{
			autoCapture = !autoCapture;
			Events["AutoCapture"].guiName = autoCapture ? "Auto Capturing" : "Manual Capturing";
		}
	// FEHLER, so toggle-Müll mal sauber definieren und überall gleich machen -> crossfeed ist zwar auch super doof definiert (im Stock)

	// hier erfolgt eine verbindung und zentrierung (ohne Drehung)
		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Capture")]
		public void Capture()
		{
			fsm.RunEvent(on_capture);
		}

	// das ist das pull-back und eine Drehung (gleichzeitig)
		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Latch")]
		public void Latch()
		{
			fsm.RunEvent(on_latch);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Release")]
		public void Release()
		{
			fsm.RunEvent(on_release);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = false, guiName = "Dock")]
		public void Dock()
		{
			DockToVessel(otherPort);

			Destroy(CaptureJoint);
			CaptureJoint = null;

			fsm.RunEvent(on_dock);
			otherPort.fsm.RunEvent(otherPort.on_dock);
		}

		public void DockToVessel(ModuleIRGF port)
		{
Vector3 position1, position2;
Transform tf; FlightCamera.TargetMode tm;

position1 = part.transform.InverseTransformPoint(FlightCamera.fetch.GetPivot().position);
position2 = part.transform.InverseTransformPoint(FlightCamera.fetch.GetCameraTransform().position);
tf = FlightCamera.fetch.Target;
tm = FlightCamera.fetch.targetMode;

			Debug.Log("Docking to vessel " + port.vessel.GetDisplayName(), gameObject);

			otherPort = port;
			dockedPartUId = otherPort.part.flightID;

			otherPort.otherPort = this;
			otherPort.dockedPartUId = part.flightID;


DockingHelper.DisableCameraSwitch();

// FEHLER, weiss nicht, ob man das hier vom DockVessels übernehmen lassen soll... aber, warum nicht, kann's ja selber tun
			if(vessel.GetTotalMass() < otherPort.vessel.GetTotalMass()) // FEHLER, ich prüf nur die Masse, "dominant Vessel" prüft noch anderes, finde ich aber nicht so gut
				DockingHelper.DockVessels(this, otherPort);
			else
				DockingHelper.DockVessels(otherPort, this);


/*
			vesselInfo = new DockedVesselInfo();
			vesselInfo.name = vessel.vesselName;
			vesselInfo.vesselType = vessel.vesselType;
			vesselInfo.rootPartUId = vessel.rootPart.flightID;

			otherPort.vesselInfo = new DockedVesselInfo();
			otherPort.vesselInfo.name = otherPort.vessel.vesselName;
			otherPort.vesselInfo.vesselType = otherPort.vessel.vesselType;
			otherPort.vesselInfo.rootPartUId = otherPort.vessel.rootPart.flightID;

			uint data = vessel.persistentId;
			uint data2 = otherPort.vessel.persistentId;

			Vessel oldvessel = vessel;

			GameEvents.onVesselDocking.Fire(data, data2);
			GameEvents.onActiveJointNeedUpdate.Fire(otherPort.vessel);
			GameEvents.onActiveJointNeedUpdate.Fire(vessel);

			if(vessel.GetTotalMass() < otherPort.vessel.GetTotalMass()) // FEHLER, ich prüf nur die Masse, "dominant Vessel" prüft noch anderes, finde ich aber nicht so gut
			{
				DockingHelper.orgInfo orgInfo = DockingHelper.BuildOrgInfo(vessel);

				Vector3 part_orgPos; Quaternion part_orgRot;
				DockingHelper.DockToVessel(port.part, port.nodeTransform, port.dockingOrientation, part, nodeTransform, dockingOrientation, snapCount, out part_orgPos, out part_orgRot);

				vessel.IgnoreGForces(10);

				DockingHelper.DisableDockingEase(otherPort.vessel);

				part.Couple(otherPort.part);

				DockingHelper.ReRootOrgInfo(orgInfo, part);
				DockingHelper.MoveOrgInfo(orgInfo, part_orgPos, part_orgRot);

				DockingHelper.ApplyOrgInfo(orgInfo, otherPort.vessel);
			}
			else
			{
				DockingHelper.orgInfo orgInfo = DockingHelper.BuildOrgInfo(otherPort.vessel);

				Vector3 otherPart_orgPos; Quaternion otherPart_orgRot;
				DockingHelper.DockToVessel(part, nodeTransform, dockingOrientation, port.part, port.nodeTransform, port.dockingOrientation, snapCount, out otherPart_orgPos, out otherPart_orgRot);

				otherPort.vessel.IgnoreGForces(10);

				DockingHelper.DisableDockingEase(vessel);

				otherPort.part.Couple(part);

				DockingHelper.ReRootOrgInfo(orgInfo, otherPort.part);
				DockingHelper.MoveOrgInfo(orgInfo, otherPart_orgPos, otherPart_orgRot);

				DockingHelper.ApplyOrgInfo(orgInfo, vessel);
			}

			GameEvents.onVesselPersistentIdChanged.Fire(data, data2);

			if(oldvessel == FlightGlobals.ActiveVessel)
			{
				FlightGlobals.ForceSetActiveVessel(vessel);
				FlightInputHandler.SetNeutralControls();
			}
			else if(vessel == FlightGlobals.ActiveVessel)
			{
				vessel.MakeActive();
				FlightInputHandler.SetNeutralControls();
			}

ahiSofort(position1, position2, position2, tm, tf);

			for(int i = 0; i < vessel.parts.Count; i++)
			{
				FlightGlobals.PersistentLoadedPartIds.Add(vessel.parts[i].persistentId, vessel.parts[i]);
				if(vessel.parts[i].protoPartSnapshot == null)
					continue;
				FlightGlobals.PersistentUnloadedPartIds.Add(vessel.parts[i].protoPartSnapshot.persistentId, vessel.parts[i].protoPartSnapshot);
			}

			GameEvents.onVesselWasModified.Fire(vessel);
			GameEvents.onDockingComplete.Fire(new GameEvents.FromToAction<Part, Part>(part, otherPort.part));

			StartCoroutine(DockingHelper.WaitAndEnableDockingEase(vessel));

*/

ahiSofort(position1, position2, position2, tm, tf);

			StartCoroutine(DockingHelper.WaitAndEnableCameraSwitch());
		}

		private void DoUndock()
		{
Vector3 position1, position2;
Transform tf; FlightCamera.TargetMode tm;

			StartCoroutine(ahi(
				position1 = part.transform.InverseTransformPoint(FlightCamera.fetch.GetPivot().position),
				position2 = part.transform.InverseTransformPoint(FlightCamera.fetch.GetCameraTransform().position),
				part.transform.InverseTransformPoint(FlightCamera.fetch.GetCameraTransform().position),
				tm = FlightCamera.fetch.targetMode, tf = FlightCamera.fetch.Target));

			if(part.parent == otherPort.part)
			{
				if(DockStatus == "Attached")
					part.decouple();
				else
					part.Undock(vesselInfo);
			}
			else
			{
				if(DockStatus == "Attached")
					otherPort.part.decouple();
				else
					otherPort.part.Undock(otherPort.vesselInfo);
			}

ahiSofort(position1, position2, position2, tm, tf);

//otherPort.DeactivateColliders(vessel);
//DeactivateColliders(otherPort.vessel);


			BuildCaptureJoint(otherPort);
			BuildCaptureJoint2();

/* FEHLER, ist glaub ich nicht nötig, weil wir danach sowieso in den latched modus gehen und somit der Joint neu gesetzt wird

				CaptureJoint.slerpDrive =
					new JointDrive
					{
						positionSpring = PhysicsGlobals.JointForce,
						positionDamper = 0f,
						maximumForce = PhysicsGlobals.JointForce
					};
*/

	// FEHLER, test, das Teil hängt sonst in der Luft??? keine Ahnung wieso?
part.AddForce(nodeTransform.forward * 0.001f);
otherPort.part.AddForce(otherPort.nodeTransform.forward * 0.001f);


/*
			otherPort.fsm.RunEvent(otherPort.on_undock);
			fsm.RunEvent(on_undock);

ahiSofort(position1, position2, position2, tm, tf);

/* -> sowas noch einbauen dann...
 * 
			if(undockPreAttached)
			{
				Decouple();
				fsm.RunEvent(on_undock);
				if(otherNode != null)
					otherNode.OnOtherNodeUndock();
				undockPreAttached = false;
				return;
			}
*/
		}

		void ahiSofort(Vector3 position, Vector3 position2, Vector3 position3, FlightCamera.TargetMode m, Transform p)
		{
			FlightCamera.fetch.SetTarget(p, true, m);

			FlightCamera.fetch.GetPivot().position = part.transform.TransformPoint(position);
			FlightCamera.fetch.SetCamCoordsFromPosition(part.transform.TransformPoint(position2));
			FlightCamera.fetch.GetCameraTransform().position = part.transform.TransformPoint(position3);
		}

static int waitframes = 1; // FEHLER, nur, damit wir sicher keine Kollisionen haben zum Testen

		IEnumerator ahi(Vector3 position, Vector3 position2, Vector3 position3, FlightCamera.TargetMode m, Transform p)
		{
// FEHLER, so lange müsste man nie warten
			for(int i = 0; i < waitframes; i++)
				yield return new WaitForEndOfFrame();

			ahiSofort(position, position2, position3, m, p);
		}

		[KSPEvent(guiActive = true, guiActiveUnfocused = true, externalToEVAOnly = true, unfocusedRange = 2f, guiName = "#autoLOC_6001445")]
		public void Undock()
		{
			Vessel oldvessel = vessel;
			uint referenceTransformId = vessel.referenceTransformId;

			DoUndock();

			fsm.RunEvent(on_undock);
			otherPort.fsm.RunEvent(otherPort.on_undock);

			if(oldvessel == FlightGlobals.ActiveVessel)
			{
				if(vessel[referenceTransformId] == null)
					StartCoroutine(WaitAndSwitchFocus());
			}
		}

		public IEnumerator WaitAndSwitchFocus()
		{
			yield return null;

Vector3 position1, position2;
Transform tf; FlightCamera.TargetMode tm;

position1 = part.transform.InverseTransformPoint(FlightCamera.fetch.GetPivot().position);
position2 = part.transform.InverseTransformPoint(FlightCamera.fetch.GetCameraTransform().position);
tm = FlightCamera.fetch.targetMode; tf = FlightCamera.fetch.Target;

			FlightGlobals.ForceSetActiveVessel(vessel);
			FlightInputHandler.SetNeutralControls();

ahiSofort(position1, position2, position2, tm, tf);
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
			vesselInfo = (dockInfo.part == (IDockable)this) ? dockInfo.vesselInfo : dockInfo.targetVesselInfo;
		}

		////////////////////////////////////////
		// IModuleInfo

		string IModuleInfo.GetModuleTitle()
		{
			return "Latching End Effector";
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
