#ifndef MCNP_INPUT_FORMAT_H
#define MCNP_INPUT_FORMAT_H

#include <vector>
#include <map>
#include <iosfwd>
#include <string>

typedef std::vector< std::string > token_list_t;

class InputDeck;

/**
 * Superclass of all cards in the input deck
 */
class Card{
protected:
  InputDeck& parent_deck;

  Card( InputDeck& deck_p ):
    parent_deck(deck_p)
  {}

  virtual ~Card(){}

public:
  InputDeck& getDeck() { return parent_deck; }

};


/**
 * Cell card
 */
class CellCard : public Card {

public:
  enum geom_token_t {INTERSECT, UNION, COMPLEMENT, LPAREN, RPAREN, CELLNUM, SURFNUM};
  typedef std::pair<enum geom_token_t, int> geom_list_entry_t;
  typedef std::vector<geom_list_entry_t> geom_list_t;
  
protected:
  int ident;
  geom_list_t geom;
  token_list_t data;
  
  CellCard( InputDeck& deck );

public:

  int getIdent() const { return ident; }
  const geom_list_t getGeom() const { return geom; }

  void print( std::ostream& s ) const; 
};

std::ostream& operator<<(std::ostream& str, const CellCard::geom_list_entry_t& t );

class AbstractSurface;

/**
 * Surface Card
 */
class SurfaceCard : public Card {
  protected:
  int ident, coord_xform;
  std::string mnemonic;
  std::vector<double> args;
  AbstractSurface* surface;

public:
  SurfaceCard( InputDeck& deck, const token_list_t tokens );

  int getIdent() const { return ident; } 
					     
  void print( std::ostream& s ) const ;

  AbstractSurface& getSurface();

};

/**
 * Data cards
 */

class DataCard : public Card {

public:
  typedef enum { TR, OTHER } kind;
  typedef std::pair< kind, int > id_t;

  DataCard( InputDeck& deck ) : Card( deck ) {}

  virtual void print( std::ostream& str ) = 0;
  virtual kind getKind(){ return OTHER; }

};


#include "geometry.hpp"

class TransformCard : public DataCard {

protected: 
  int ident;
  Transform trans;

public:
  TransformCard( InputDeck& deck, int ident_p, bool degree_format, const token_list_t& input );

  const Transform& getTransform() const{ return trans; } 

  virtual void print( std::ostream& str );
  virtual kind getKind(){ return TR; }

};



/**
 * Main interface to MCNP reader: the InputDeck 
 */
class InputDeck{

public:
  typedef std::vector< CellCard* > cell_card_list;
  typedef std::vector< SurfaceCard* > surface_card_list;
  typedef std::vector< DataCard* > data_card_list;

protected:
  class LineExtractor;

  cell_card_list cells;
  surface_card_list surfaces;
  data_card_list datacards;

  std::map<int, CellCard*> cell_map;
  std::map<int, SurfaceCard*> surface_map;
  std::map< DataCard::id_t, DataCard*> datacard_map;

  void parseTitle( LineExtractor& lines );
  void parseCells( LineExtractor& lines );
  void parseSurfaces( LineExtractor& lines );
  void parseDataCards( LineExtractor& lines );

public:

  ~InputDeck();

  cell_card_list& getCells() { return cells; }
  surface_card_list& getSurfaces() { return surfaces; } 
  data_card_list& getDataCards(){ return datacards; }

  CellCard* lookup_cell_card(int ident){
    return (*cell_map.find(ident)).second;
  }

  SurfaceCard* lookup_surface_card(int ident){
    return (*surface_map.find(ident)).second;
  }

  DataCard* lookup_data_card( const DataCard::id_t& ident ){
    return (*datacard_map.find(ident)).second;
  }

  DataCard* lookup_data_card( DataCard::kind k, int ident ){
    return lookup_data_card( std::make_pair( k, ident ) );
  }

  static InputDeck& build( std::istream& input );
  
  void createGeometry();


};

template < class T >
std::ostream& operator<<( std::ostream& out, const std::vector<T>& list );

#endif /* MCNP_INPUT_FORMAT_H */
