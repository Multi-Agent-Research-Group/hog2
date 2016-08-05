//
//  AirplaneTicketAuthority.cpp
//  hog2 glut
//
//  Created by David Chan on 7/24/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "AirplaneTicketAuthority.h"

uint64_t RAirspaceRecord::GetNumUniqueTicketsAtTime(float time) 
{   
    ITree::BasicInterval* x = new ITree::BasicInterval(time-FLT_EPSILON,time+FLT_EPSILON);
    std::vector<ITree::Interval*> tkts = this->issued_tickets->GetOverlappingIntervals(x);
    delete x;
    std::set<int> uns;
    for (int i = 0; i < tkts.size(); i++) {
        RAirspaceTicket * cticket = (RAirspaceTicket *) tkts[i];
        uns.insert(cticket->issued_unit);
    }
    return uns.size();
}

uint64_t RAirspaceRecord::GetNumUniqueTicketsAtTime(float time_begin, float time_end) 
{
    ITree::BasicInterval* x = new ITree::BasicInterval(time_begin-FLT_EPSILON,time_end+FLT_EPSILON);
    std::vector<ITree::Interval*> tkts = this->issued_tickets->GetOverlappingIntervals(x);
    delete x;
    std::set<int> uns;
    for (int i = 0; i < tkts.size(); i++) {
        RAirspaceTicket * cticket = (RAirspaceTicket *) tkts[i];
        uns.insert(cticket->issued_unit);
    }
    return uns.size();
}


// Methods for registering restricted airspace with the ticket authority
RAirspaceRecord& TicketAuthority::RegisterAirspace(const airtimeState &loc2) {
    RAirspaceRecord r(airConstraint(loc2), 1);
    restricted_airspace.push_back(r);
    return restricted_airspace.back();
}
RAirspaceRecord& TicketAuthority::RegisterAirspace(const airtimeState &loc1, const airtimeState &loc2) {
    RAirspaceRecord r(airConstraint(loc1, loc2), 1);
    restricted_airspace.push_back(r);
    return restricted_airspace.back();
}
RAirspaceRecord&  TicketAuthority::RegisterAirspace(const airtimeState &loc2, uint64_t max_capacity) {
    RAirspaceRecord r(airConstraint(loc2), max_capacity);
    restricted_airspace.push_back(r);
    return restricted_airspace.back();
}
RAirspaceRecord&  TicketAuthority::RegisterAirspace(const airtimeState &loc1, const airtimeState &loc2, uint64_t max_capacity) {
    RAirspaceRecord r(airConstraint(loc1, loc2), max_capacity);
    restricted_airspace.push_back(r);
    return restricted_airspace.back();
}

// Tools for getting tickets
bool TicketAuthority::CanObtainTicket(airtimeState &nodeID) {

    CleanupIssued(nodeID);
    bool valid = true;
    for (RAirspaceRecord& rasp : restricted_airspace) {
        // Check first if the node is in the airspace
        if (rasp.governed_area.ConflictsWith(nodeID)) {
            // If it is in the airspace, we need to figure out how many units
            // are in the space right now
            uint64_t cur_units = rasp.GetNumUniqueTicketsAtTime(nodeID.t);
            if (cur_units < rasp.max_capacity) {
                valid = true;
            } else if (nodeID.current_tickets.size() > 0) { // Maybe we already have a valid ticket for this time
                for (RAirspaceTicket* tic : nodeID.current_tickets) {
                    if (tic->issuing_airspace == &rasp && tic->GetLowPoint() - FLT_EPSILON < nodeID.t && tic->GetHighPoint() + FLT_EPSILON > nodeID.t) {
                        valid = true;
                    }
                }
                valid = false;
            } else { // We don't have a ticket and we can't get one
                valid = false;
            }
        }
    }
    // We're not moving into in any restricted airspace
    return valid;
}

bool TicketAuthority::IssueTicket(airtimeState &nodeID, int unit) {
    CleanupIssued(nodeID);
    if (CanObtainTicket(nodeID)) {
        for (RAirspaceRecord& rasp : restricted_airspace) {
            // Check first if the node is in the airspace
            if (rasp.governed_area.ConflictsWith(nodeID)) {
                // Create a ticket
                RAirspaceTicket* t = new RAirspaceTicket(&rasp, unit, next_ticket_id, nodeID.t, nodeID.t + EXPIRATION_TIME, ticket_iteration);
                next_ticket_id += 1;
                // Add it to the airspace
                t->itree_loc = rasp.issued_tickets->Insert(t);
                // Add it to the state
                nodeID.current_tickets.push_back(t);
            }
        }
        return true;
    } else {
        return false;
    }
}

void TicketAuthority::IssueTicketIgnoringCheck(airtimeState &nodeID, int unit) {
    CleanupIssued(nodeID);
    for (RAirspaceRecord& rasp : restricted_airspace) {
        // Check first if the node is in the airspace
        if (rasp.governed_area.ConflictsWith(nodeID)) {
            // Create a ticket
            RAirspaceTicket* t = new RAirspaceTicket(&rasp, unit, next_ticket_id, nodeID.t, nodeID.t + EXPIRATION_TIME, ticket_iteration);
            next_ticket_id += 1;
            // Add it to the airspace
            t->itree_loc = rasp.issued_tickets->Insert(t);
            // Add it to the state
            nodeID.current_tickets.push_back(t);
        }
    }
}

// Do try not to use this function. It's probably not necessary
void TicketAuthority::ReleaseTicket(airtimeState &nodeID, RAirspaceTicket* ticket) {
    CleanupIssued(nodeID);
    // Release it from the issuing airspace
    // Can't do that yet
    // Release it from the current node
    int i;
    for (i = 0; i < nodeID.current_tickets.size(); i ++) {
        if (nodeID.current_tickets[i] == ticket)
            break;
    }
    nodeID.current_tickets.erase (nodeID.current_tickets.begin()+i);
}


bool TicketAuthority::IssueTicketsForPath(std::vector<airtimeState>& path, int unit) {
    bool valid = true;
    for (airtimeState& x : path) {
        valid = valid && IssueTicket(x, unit);
    }
    return valid;
}

void TicketAuthority::IssueTicketsForPathIgnoringCheck(std::vector<airtimeState>& path, int unit) {
    for (airtimeState& x : path) {
        IssueTicketIgnoringCheck(x, unit);
    }
}


void TicketAuthority::UnissueAllTickets() {
    for (RAirspaceRecord& rasp : restricted_airspace) {
        delete rasp.issued_tickets;
        rasp.issued_tickets = new IntervalTree();
    }
    next_ticket_id = 0;
    ticket_iteration += 1;
}

void TicketAuthority::CleanupIssued(airtimeState& nodeID) {
     uint64_t iter = this->ticket_iteration;
     auto new_end = std::remove_if(nodeID.current_tickets.begin(), nodeID.current_tickets.end(),
        [iter](RAirspaceTicket* tic) { return tic->issued_iteration != iter; }
     ); 
     nodeID.current_tickets.erase(new_end, nodeID.current_tickets.end());
}
