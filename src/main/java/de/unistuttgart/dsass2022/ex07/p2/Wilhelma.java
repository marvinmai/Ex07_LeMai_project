package de.unistuttgart.dsass2022.ex07.p2;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Iterator;

import org.apache.commons.io.IOUtils;
import org.json.JSONArray;
import org.json.JSONObject;

public class Wilhelma {
	
	
	/**
	 * This class contains the main method, which starts the program.
	 * I.e. it reads the whilhelma.osm into an JSONObject from which it generates the WeightedGraph.
	 * 
	 * Then it creates a new shortest path object, i.e. executes the dijkstra.
	 * 
	 * Finally the shortest path between the defined nodes is exported into geojson and printed to the console.
	 * You can view this geojson file e.g. on "geojson.io".
	 * 
	 * Here is a short list of POI's you can experiment with:
	 * 
	 * Haupteingang: 3739803960
	 * Eingang Rosensteinpark: 258355787
	 * Nebeneingang Pragstraße: 663604553
	 * 
	 * Giraffenhaus: 655393713
	 * Maurisches Landhaus: 663516727
	 * Aquarien: 663593126
	 * Damaszenerhalle: 663593128
	 * Insektarium: 663593130
	 * Raubkatzen: 663597121
	 * Amazonienhaus: 663597129
	 * Dickhaeuter: 663597175
	 * Belvedere: 663609565
	 * Suedamerikahaus: 4094951965
	 * Gewaechshaeuser: 9210287481
	 * Restaurant Amazonica: 4094951957
	 * 
	 * 
	 * 
	 * 
	 * @param args
	 * @throws IOException
	 */
	public static void main(String[] args) throws IOException {
		
		
		
		
		long start = 3739803960L;
        long destination = 663516727L;
		
		
		
		
		
		WeightedGraph graph = new WeightedGraph();
		
		
		String dir = "src/main/resources/wilhelma.osm";
        InputStream is = new FileInputStream(dir);
        String jsonTxt = IOUtils.toString(is, "UTF-8");
        JSONObject json = new JSONObject(jsonTxt);  
        
        //iterates over all "node"-types and adds the ID, lat and long to the graph
        for(Object s: json.getJSONArray("elements")) {
        	JSONObject entry = (JSONObject) s;
        	if(((String) entry.get("type")).compareTo("node") == 0) {
        		long id = Long.valueOf(entry.get("id").toString());
        		double lat = (double) entry.get("lat");
        		double lon = (double) entry.get("lon");
        		graph.addNode(id, new Node(id,lat,lon));
        	} 
        	
        }
        
        //iterates over all "way"-types and adds them to the graph
        for(Object s: json.getJSONArray("elements")) {
        	JSONObject entry = (JSONObject) s;
        	
        	if (((String) entry.get("type")).compareTo("way") == 0) {
        		JSONArray wayPoints = entry.getJSONArray("nodes");
        		for(int i = 0; i < wayPoints.length()-1; i++) {
        			long src = Long.valueOf(wayPoints.get(i).toString());
        			long dst =Long.valueOf(wayPoints.get(i+1).toString());
        			
        			Node srcMeta = graph.getNode(src);
        			Node dstMeta = graph.getNode(dst);
        			//approximate the weight of the edge by the pythagoran theorem. For a better approx. have a look at geodesics
        			double weight = Math.sqrt(Math.pow(srcMeta.getLatitude()-dstMeta.getLatitude(),2)
        					+Math.pow(srcMeta.getLongitude()-dstMeta.getLongitude(),2));
        			
        			//OSM-data is undirected
        			graph.addEdge(src,dst,weight);
        			graph.addEdge(dst,src,weight);
        		}
        	}
        }
        
        
        ShortestPath sp = new ShortestPath(graph, start);
        
        
        //generating the geojson. i.e. iterating over the path and getting the coordinates of the nodes on it
        String path = "";

        Iterator<IEdge> iter = sp.pathTo(destination).iterator();
        while(iter.hasNext()){
        	IEdge edge = iter.next();
        	long src = edge.getSource();
        	path += "["+ graph.getNode(src).getLongitude()+","+graph.getNode(src).getLatitude()+"],\n";
        	
        	if(!iter.hasNext()) {
        		long dst = edge.getDestination();
        		path += "    ["+ graph.getNode(dst).getLongitude()+","+graph.getNode(dst).getLatitude()+"]\n";
        	}
        }
        
        path = "{\n"
      		+ " \"type\": \"FeatureCollection\",\n"
      		+ "  \"features\": [\n"
      		+ "  {\"type\": \"Feature\",\n"
      		+ "   \"geometry\": {\n"
      		+ "    \"type\": \"LineString\",\n"
      		+ "     \"coordinates\": [\n"
      		+ path
      		+ "    ]},\n"
      		+ "    \"properties\": {\n"
      		+ "    }\n  "
      		+ "   },"
      		+ "   {\n"
      		+ "      \"type\": \"Feature\",\n"
      		+ "       \"geometry\": {\n"
      		+ "        \"type\": \"Point\",\n"
      		+ "         \"coordinates\": ["+graph.getNode(start).getLongitude()+","+ graph.getNode(start).getLatitude()+"]\n"
      		+ "           },\n"
      		+ "           \"properties\": {\n"
      		+ "               \"marker-symbol\": \"marker\"\n"
      		+ "           }\n"
      		+ "   },"
      		+ "   {\n"
      		+ "      \"type\": \"Feature\",\n"
      		+ "       \"geometry\": {\n"
      		+ "        \"type\": \"Point\",\n"
      		+ "         \"coordinates\": ["+graph.getNode(destination).getLongitude()+","+ graph.getNode(destination).getLatitude()+"]\n"
      		+ "           },\n"
      		+ "           \"properties\": {\n"
      		+ "               \"marker-symbol\": \"square\"\n"
      		+ "           }\n"
      		+ "       } "
      		+ "  ]\n"
    		+ "}";
        
        
        
        
        
        
        
        System.out.println(path);


	}
}
