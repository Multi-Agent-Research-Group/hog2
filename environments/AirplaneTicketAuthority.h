//
//  Airplane.h
//  hog2 glut
//
//  Created by David Chan on 7/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirplaneTicketAuthority_h
#define AirplaneTicketAuthority_h

#define EXPIRATION_TIME 2.0f

#include "Unit.h"
#include "IntervalTree.h"
#include "AirStates.h"

#include <float.h>
#include <set>

struct RAirspaceTicket;

// Struct governing the restricted airspace
struct RAirspaceRecord {
	RAirspaceRecord(airConstraint g, uint64_t max_capacity) : governed_area(g), max_capacity(max_capacity) {issued_tickets = new IntervalTree();}
	RAirspaceRecord(airConstraint g, uint64_t max_capacity, const std::vector<RAirspaceRecord*>& rtickets) : governed_area(g), max_capacity(max_capacity), entry_airspaces(rtickets) {issued_tickets = new IntervalTree();}
	
	airConstraint governed_area;

	std::vector<RAirspaceRecord*> entry_airspaces;
	IntervalTree* issued_tickets;
	uint64_t max_capacity;

	uint64_t GetNumUniqueTicketsAtTime(float time);
	uint64_t GetNumUniqueTicketsAtTime(float time_begin, float time_end);

};

// Struct which governs entry to a restricted airspace
struct RAirspaceTicket : public ITree::Interval{
	RAirspaceTicket(RAirspaceRecord* a, int e, uint64_t b, float ent_time, float ext_time, uint64_t it) : issuing_airspace(a), issued_unit(e), ticket_ID(b), entry_time(ent_time), expiration_time(ext_time), issued_iteration(it) {}
	RAirspaceRecord* issuing_airspace;
	int issued_unit;
	uint64_t ticket_ID;
	float entry_time;
	float expiration_time;
	ITree::ITNode* itree_loc;
	uint64_t issued_iteration;

	virtual float GetLowPoint() {return entry_time;}
	virtual float GetHighPoint() {return expiration_time;}
	void Print() {} // Do nothing for now

	virtual ~RAirspaceTicket() {}
};



class TicketAuthority {
	public:

		TicketAuthority() : ticket_iteration(0),next_ticket_id(1) {}

		// Tools for registering airspace with the authority
		RAirspaceRecord& RegisterAirspace(const airtimeState &loc2);
		RAirspaceRecord& RegisterAirspace(const airtimeState &loc1, const airtimeState &loc2);
		RAirspaceRecord& RegisterAirspace(const airtimeState &loc2, uint64_t max_capacity);
		RAirspaceRecord& RegisterAirspace(const airtimeState &loc1, const airtimeState &loc2, uint64_t max_capacity);

		// Tools for getting tickets
		bool CanObtainTicket(airtimeState &nodeID);
		bool IssueTicket(airtimeState &nodeID, int unit);
		void IssueTicketIgnoringCheck(airtimeState &nodeID, int unit);
		void ReleaseTicket(airtimeState &nodeID, RAirspaceTicket* ticket);
		bool IssueTicketsForPath(std::vector<airtimeState>& path, int unit);
		void IssueTicketsForPathIgnoringCheck(std::vector<airtimeState>& path, int unit);

		// Tools for resetting the ticket authority
		void UnissueAllTickets();
		void CleanupIssued(airtimeState& nodeID);


		const std::vector<RAirspaceRecord>& GetRestrictedAirspace() {return restricted_airspace;};


	private:
		std::vector<RAirspaceRecord> restricted_airspace;
		uint64_t next_ticket_id;
		uint64_t ticket_iteration;
};




#endif /* AirplaneTicketAuthority_h */


